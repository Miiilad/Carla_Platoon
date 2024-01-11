import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize

import gurobipy as gp
from gurobipy import GRB


class Control():
    def __init__(self, h, t_end, prediction_H, control_H, Objective, Model):
        self.Objective = Objective
        self.h = h
        self.kf = int(20 / h)
        self.p_H = prediction_H
        self.c_H = control_H
        self.Model = Model
        self.Objective = Objective

        A = np.array([[0, 1, -1.6],
                      [0, 0, -1],
                      [0, 0, -2.17391304]])
        B = np.array([[0],
                      [0],
                      [-1.59130435]])
        self.Ad = A * h + np.eye(3)
        self.Bd = B * h

        self.x_predict_sequence=[]

    def calculate(self, x, u_lim, k):
        self.xp = np.copy(x)

        #################
        m = gp.Model("qp")
        # m.params.NonConvex = 2
        Up = m.addMVar(shape=(self.Model.dim_m, self.c_H), lb=list(u_lim[0]*np.ones((self.Model.dim_m, self.c_H))),
                       ub=list(u_lim[1]*np.ones((self.Model.dim_m, self.c_H))), name="U")
        Xp = m.addMVar(shape=(self.Model.dim_n, self.p_H), lb=list(-10 * np.ones((self.Model.dim_n, self.p_H))),
                       ub=list(5 * np.ones((self.Model.dim_n, self.p_H))), name="Xp")

        # This needs to be improved. It won't work for self.Model_m != 1
        # Create the control sequence considering the repeated tail
        if self.p_H > self.c_H:
            Up_joined = self.getUp_joined(Up)
        elif self.p_H == self.c_H:
            Up_joined = Up
        else:
            raise SyntaxError('Prediction horizon must be >= Control horizon')

        # Define the objective of the optimizer
        obj = self.opt_obj_mat(Xp, Up_joined)
        # Define the constratint of the optimizer
        self.opt_const_mat(m, Xp, Up_joined)

        # #STL constraint for F_[0,20] |xf-[3,0,0]|< r   (F ~ means Eventually)
        # r=1
        # b0=114
        # a0 = (r-b0) / 20
        # xf=np.array([3,0,0])
        # etta=0.5 #0<etta<1
        # for n in range(k,min([k+self.p_H,self.kf])-1):
        #     # print(n-k,k,min([k+self.p_H,self.kf])-1)
        #     tn=self.h*n
        #     tn_1=self.h*(n+1)
        #     gamma=a0*tn+b0
        #     gamma_1 = a0 * tn_1 + b0
        #     en=Xp[:,n-k]-xf
        #     en_1=Xp[:,n+1-k]-xf
        #     m.addConstr(gamma_1-np.inner(en_1,en_1)+(etta-1)*(gamma-np.inner(en,en)) >= 0)
        #     m.addConstr(gamma-np.inner(en,en)>=0)

        # Configure the optimizer and Solve
        m.setObjective(obj, GRB.MINIMIZE)
        m.Params.LogToConsole = 0
        m.Params.FeasibilityTol = 1e-3
        m.params.NonConvex = 2
        m.optimize()

        # Extract the variables u
        # var = [v.x for v in m.getVars()]
        # m.write("myLP.lp")
        # print(var)
        var_u = m.getVars()
        u = np.array(var_u[0].x)

        start_ind=self.c_H * self.Model.dim_m
        # for v in m.getVars()[start_ind:start_ind+self.p_H]:
        #     print(f"{v.VarName} = {v.X}")
        x_predict=[v.x for v in m.getVars()[start_ind:start_ind + self.p_H]]
        # print(x_predict)
        self.x_predict_sequence=x_predict[1:]


        ####################

        u = np.clip(u, u_lim[0], u_lim[1])
        # print('u',u)
        return u

    def opt_obj(self, Up):
        self.Objective.resetSum()
        for p in range(self.p_H):
            self.Objective.getSum(self.xp, Up[p])
            self.xp = self.Model.nextDiscrete(self.xp, Up[p])
        for p in range(self.p_H, self.p_H):
            self.Objective.getSum(self.xp, Up[-1])
            self.xp = self.Model.nextDiscrete(self.xp, Up[-1])
        return self.Objective.sum

    def opt_obj_mat(self, Xp, Up):
        o1 = (Xp.T @ self.Objective.Q @ Xp)
        o2 = (Up.T @ self.Objective.R @ Up)
        obj = sum([o1[i, i] + o2[i, i] for i in range(self.p_H)])
        return obj

    def opt_const_mat(self, m, Xp, Up):
        m.addConstr(Xp[:, 0] == self.xp)
        m.addConstr(Xp[:, 1:] == self.Ad @ Xp[:, :-1] + self.Bd @ Up[:, :-1])


    def getUp_joined(self, Up):
        Up_tail = Up[:, -1].tolist()
        Up_tail *= self.p_H - self.c_H
        Up_joined = Up.tolist()[0] + Up_tail

        # Reshape and convert back to an MVar
        Up_joined = np.array(Up_joined).reshape(self.Model.dim_m, self.p_H)
        Up_joined = Up_joined.tolist()
        Up_joined = gp.MVar.fromlist(Up_joined)

        return Up_joined


class Objective():
    def __init__(self, Q, R):
        self.Q = Q
        self.R = R
        self.sum = 0

    def stage_cost(self, x, u):
        return np.matmul(np.matmul(x, self.Q), x) + u * self.R * u

    def getSum(self, x, u):
        self.sum += self.stage_cost(x, u)
        return self.sum

    def resetSum(self):
        self.sum = 0


class SimResults():
    def __init__(self, t, Ctrl, Model, output_dir_path, select={'states': 1}):
        self.t = t
        len_t = len(t)
        self.Model = Model
        self.Ctrl = Ctrl
        self.x_s_history = np.zeros((self.Model.dim_n, len_t))
        self.u_history = np.zeros((self.Model.dim_m, len_t))

        # the residule of x, attack time
        self.residule = np.zeros(len_t)
        self.attack_time = -1

        self.select = select
        self.output_dir_path = output_dir_path
        self.pallet = ['r', 'g', 'b', 'm', '#E67E22', '#1F618D']

    def record_state(self, i, x, u, x_ref):
        temp = np.copy(x)
        temp = temp - x_ref
        self.x_s_history[:, i] = temp
        self.u_history[:, i] = u

    def graph(self, j, i):
        # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>PLOT<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        # plot the 'control' + 'states' of the system vs. 'time' ################################
        if self.select['states']:
            fig1 = plt.figure()
            for im in range(self.Model.dim_m):
                plt.plot(self.t[:i], self.u_history[im, :i], 'c')
            for ii in range(self.Model.dim_n):
                plt.plot(self.t[:i], self.x_s_history[ii, :i],
                         self.pallet[ii % len(self.pallet)])

            plt.legend(
                ["Control", "Position", "Velocity", "Accelration"], loc=1)
            plt.xlabel('t (sec)')
            plt.ylabel('States and Control')
            # plt.tight_layout()
            # plt.ylim((-5, 5))

            plt.grid(color='k', linestyle=':', linewidth=1)
            plt.savefig(self.output_dir_path +
                        '/fig_states_control{}.pdf'.format(j), format='pdf')
            # plt.close(fig1)
            # plt.show()

    # record everything in the movie
    def record_system(self, i, x, u, x_ref, residule, attack_time=-1):
        if attack_time != -1:
            self.attack_time = attack_time
        temp = np.copy(x)
        temp = temp - x_ref
        self.x_s_history[:, i] = temp
        self.u_history[:, i] = u
        self.residule[i] = residule

    def graph_system(self, j, i):
        # initialize
        fig, ax = plt.subplots()

        # plot the control and the system
        # plot state variable and input
        for im in range(self.Model.dim_m):
            ax.plot(self.t[:i], self.u_history[im, :i], 'c')
        for ii in range(self.Model.dim_n):
            ax.plot(self.t[:i], self.x_s_history[ii, :i],
                    self.pallet[ii % len(self.pallet)])

        fig.legend(
            ["Control", "Position", "Velocity", "Accelration"], loc=1)
        plt.xlabel('t (sec)')
        plt.ylabel('States and Control')

        if self.attack_time > 0:
            plt.axvline(x=self.attack_time, color='r', linewidth=0.15)
            plt.text(self.attack_time, 0, s="attack launched", color='r')

        plt.grid(color='k', linestyle=':', linewidth=1)
        plt.savefig(self.output_dir_path +
                    '/fig_system{}.pdf'.format(j), format='pdf')

    def graph_system(self, j, i):
        # initialize
        fig, ax = plt.subplots()

        # plot the control and the system
        # plot state variable and input
        for im in range(self.Model.dim_m):
            ax.plot(self.t[:i], self.u_history[im, :i], 'c')
        for ii in range(self.Model.dim_n):
            ax.plot(self.t[:i], self.x_s_history[ii, :i],
                    self.pallet[ii % len(self.pallet)])

        fig.legend(
            ["Control", "Position", "Velocity", "Accelration"], loc=1)
        plt.xlabel('t (sec)')
        plt.ylabel('States and Control')

        if self.attack_time > 0:
            plt.axvline(x=self.attack_time, color='r', linewidth=0.15)
            plt.text(self.attack_time, -0.05, "attack launched",
                     transform=ax.get_xaxis_transform(),
                     ha='center', va='top', color='r')

        plt.grid(color='k', linestyle=':', linewidth=1)
        plt.savefig(self.output_dir_path +
                    '/fig_system{}.pdf'.format(j), format='pdf')

    def graph_residule(self, j, i):
        # initialize
        fig, ax = plt.subplots()

        # plot the control and the system
        # plot state variable and input
        ax.plot(self.t, self.residule, 'c')

        fig.legend(["residule"], loc=1)
        plt.xlabel('t (sec)')
        plt.ylabel('residule value')

        if self.attack_time > 0:
            plt.axvline(x=self.attack_time, color='r', linewidth=0.15)
            # plt.text(self.attack_time, 0, s="attack launched", color='r')
            plt.text(self.attack_time, -0.05, "attack launched",
                     transform=ax.get_xaxis_transform(),
                     ha='center', va='top', color='r')
        plt.grid(color='k', linestyle=':', linewidth=1)
        plt.savefig(self.output_dir_path +
                    '/fig_residule{}.pdf'.format(j), format='pdf')

    def printout(self, j):
        # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>PRINT<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        pass