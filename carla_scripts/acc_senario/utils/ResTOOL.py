import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize

import gurobipy as gp
from gurobipy import GRB


class Control():
    def __init__(self, h, prediction_H, control_H, Objective):
        self.Objective = Objective
        self.h = h
        self.kf = int(20 / h)
        self.p_H = prediction_H
        self.c_H = control_H
        self.dim_m=1
        self.dim_n=3

        self.Objective = Objective

        A = np.array([[0, 1, -1.6],
                      [0, 0, -1],
                      [0, 0, -2.17391304]])
        B = np.array([[0],
                      [0],
                      [1.59130435]])
        H=np.array([[0],
                    [1],
                    [0]])
        self.Ad = A * h + np.eye(3)
        self.Bd = B * h
        self.Hd= H * h 

        self.x_predict_sequence=[]
        self.u_pre = 0.0

    def calculate(self, x,v_dot_lead, u_lim):
        self.xp = np.copy(x)
        self.v_dot_lead=v_dot_lead

        #################
        m = gp.Model("qp")
        # m.params.NonConvex = 2
        Up = m.addMVar(shape=(self.dim_m, self.c_H), lb=list(u_lim[0]*np.ones((self.dim_m, self.c_H))),
                       ub=list(u_lim[1]*np.ones((self.dim_m, self.c_H))), name="U")
        Xp = m.addMVar(shape=(self.dim_n, self.p_H), lb=list(-100 * np.ones((self.dim_n, self.p_H))),
                       ub=list(100 * np.ones((self.dim_n, self.p_H))), name="Xp")

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
        # print(var_u)
        try:
            u = np.array(var_u[0].x)
        except:
            u = self.u_pre
            print("WARNING!!! >>>> MPC infeasible <<<<<")

        # start_ind=self.c_H * self.dim_m
        # # for v in m.getVars()[start_ind:start_ind+self.p_H]:
        # #     print(f"{v.VarName} = {v.X}")
        # x_predict=[v.x for v in m.getVars()[start_ind:start_ind + self.p_H]]
        # # print(x_predict)
        # self.x_predict_sequence=x_predict[1:]


        ####################

        u = np.clip(u, u_lim[0], u_lim[1])
        # print('u',u)
        self.u_pre = u
        return u
    def eval_nominal(self,x,u,v_dot_lead):
        x=np.array(x)
        x_next=self.Ad @ x + (self.Bd * u + self.Hd * v_dot_lead).flatten()
        return x_next
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
        o3 = 100*(Up[:,1:].T - Up[:,:-1].T)  @ self.Objective.R @ (Up[:,1:] - Up[:,:-1]) 
        obj = sum([o1[i, i] + o2[i, i] for i in range(self.p_H)])+sum([o3[i, i] for i in range(self.c_H)])
        return obj

    def opt_const_mat(self, m, Xp, Up):
        m.addConstr(Xp[:, 0] == self.xp)
        m.addConstr(Xp[:, 1:] == self.Ad @ Xp[:, :-1] + self.Bd @ Up[:, :-1] + self.v_dot_lead* self.Hd @ np.ones((1,self.p_H-1)))


    def getUp_joined(self, Up):
        Up_tail = Up[:, -1].tolist()
        Up_tail *= self.p_H - self.c_H
        Up_joined = Up.tolist()[0] + Up_tail

        # Reshape and convert back to an MVar
        Up_joined = np.array(Up_joined).reshape(self.dim_m, self.p_H)
        Up_joined = Up_joined.tolist()
        Up_joined = gp.MVar.fromlist(Up_joined)

        return Up_joined
    
    def Safe_Control(self,net,x,u,v_dot_lead,u_lim):
        m = gp.Model("qp")
        # m.params.NonConvex = 2
        Us = m.addMVar(shape=(self.dim_m, self.dim_m), lb=list(u_lim[0]*np.ones((self.dim_m, self.dim_m))),
                       ub=list(u_lim[1]*np.ones((self.dim_m, self.dim_m))), name="U")
        
        # Define the objective function
        objective = (u - Us) * (u - Us)
        m.Params.LogToConsole = 0
        m.Params.FeasibilityTol = 1e-3
        m.setObjective(objective, GRB.MINIMIZE)
        etta = 0.1

        y=net.evaluate(x,u)
        bx,ax = net.partial_derivative_u(x)
        # print(" NN:",y)
        # print("Gradient:", bx,ax)
        # print("a(x)+b(x)u:", ax+bx*u,'\n')

        f_bar = self.eval_nominal(x,u,v_dot_lead)
        f_bar=np.array(f_bar).reshape(self.dim_n,1)
        Bx=self.BF(x)
        Bx_next=self.BF(f_bar+ ax+bx @ Us)

        m.addConstr(Bx_next -Bx >= -etta * Bx, "constraint")

        try:
            # Solve the model
            m.optimize()

            # Get the optimal value of Us
            optimal_Us = Us.X
            print("Unsafe control",u,"Optimal value of U safe:", optimal_Us)
            out=optimal_Us.reshape(self.dim_m)[0]
        except:
            print("Unsafe control",u,"safe control is infeasibe!")
            out=u
        return out


    def BF(self,x):
        W=np.diag([1,0,0.1])
        C=3
        if isinstance(x, np.ndarray):
            out = C**2 - x.T @ W @ x  
        else:
            # Compute the quadratic expression manually
            quad_expr = gp.quicksum(W[i][i] * x[i] * x[i] for i in range(self.dim_n))

            # Define the complete expression
            out = C**2 - quad_expr  # Assuming C is a constant 
        return out


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
        self.x_s_history = np.zeros((self.dim_n, len_t))
        self.u_history = np.zeros((self.dim_m, len_t))

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
            for im in range(self.dim_m):
                plt.plot(self.t[:i], self.u_history[im, :i], 'c')
            for ii in range(self.dim_n):
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
        for im in range(self.dim_m):
            ax.plot(self.t[:i], self.u_history[im, :i], 'c')
        for ii in range(self.dim_n):
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
        for im in range(self.dim_m):
            ax.plot(self.t[:i], self.u_history[im, :i], 'c')
        for ii in range(self.dim_n):
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