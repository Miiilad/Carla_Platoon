import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize

import gurobipy as gp
from gurobipy import GRB
import os

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

    def calculate(self, x,v_dot_lead,u_pre, u_lim):
        self.xp = np.copy(x)
        self.v_dot_lead=v_dot_lead
        self.u_pre=u_pre.reshape(self.dim_m,1)

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
        x_next=(self.Ad @ x).reshape(self.dim_n,1) + (self.Bd * u + self.Hd * v_dot_lead)
        return x_next
    
    def eval_nominal_vehicle(self,x,u):
        
        A = np.array([[0, 1, -1.6],
                      [0, 0, -1],
                      [0, 0, -2.17391304]])
        B = np.array([[0],
                      [0],
                      [1.59130435]])
        Ad = A * self.h + np.eye(3)
        Bd = B * self.h
        x=np.array(x)
        x_next=(Ad @ x).reshape(self.dim_n,1) + (Bd * u )
        return x_next.reshape(self.dim_n)
    
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
        u_rate_weight= 100 * self.Objective.R
        o1 = (Xp.T @ self.Objective.Q @ Xp)
        o2 = (Up.T @ self.Objective.R @ Up)
        o3 = (Up[:,1:].T - Up[:,:-1].T)  @ u_rate_weight @ (Up[:,1:] - Up[:,:-1]) 
        o4 = (Up[:,0].T - self.u_pre.T)  @ u_rate_weight @ (Up[:,0] - self.u_pre)
        obj = sum([o1[i, i] + o2[i, i] for i in range(self.p_H)])+sum([o3[i, i] for i in range(self.c_H)])+o4
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
        Us = m.addMVar(shape=(self.dim_m, 1), lb=list(u_lim[0]*np.ones((self.dim_m, self.dim_m))),
                       ub=list(u_lim[1]*np.ones((self.dim_m, self.dim_m))), name="U")
        
        # Define the objective function
        objective = (u - Us) * (u - Us)
        m.Params.LogToConsole = 0
        m.Params.FeasibilityTol = 1e-3
        m.setObjective(objective, GRB.MINIMIZE)
        etta = 0.6

        y=net.evaluate(x,u)
        bx,ax = net.partial_derivative_u(x)
        # print(" NN:",y)
        # print("Gradient:", bx,ax)
        # print("a(x)+b(x)u:", ax+bx*u,'\n')
        f_bar = self.eval_nominal(x,Us,v_dot_lead)#+ ax+bx @ Us

        # f_bar=np.array(f_bar).reshape(self.dim_n,1)
        Bx=self.BF(x)
        #commented for now
        # Bx_next=self.BF(f_bar+ ax+bx @ Us)
        # Bx_next=self.BF(f_bar+ 0*ax+0*bx @ Us)
        x_dot=1/self.h*(f_bar-x.reshape(self.dim_n,1))
        # print(self.barrier_higher_degree(x,x_dot,v_dot_lead))
    

        # m.addConstr(Bx_next -Bx >= -etta * Bx, "constraint")
        m.addConstr(self.barrier_higher_degree(x,x_dot,v_dot_lead) >= 0, "constraint")

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
        '''
        barrier function
        '''
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
    
    def barrier_higher_degree(self,x,x_dot,v_dot_lead):
        d_th=5
        
        dd=x[0]
        dv=x[1]
        a=x[2]
        
        dv_dot=v_dot_lead-a
        a_dot=x_dot[2]
        
        k1,k2,k3=[0.005,0.003,0.001]
        
        phi0=d_th**2-dd**2
        phi1=-2*dv*dd + k1*phi0
        phi2=-2*dv_dot*dd-2*dv**2+k2*phi1
        phi3=2*a_dot*dd-2*dv_dot*dv-4*dv_dot*dv+k3*phi2
        
        return phi3


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
    def __init__(self, labels, max_length=10000, output_dir_path="./Simulation_Results", select={'states': 1}):
        self.max_length = max_length
        self.labels = labels  # Mixed list of strings and lists
        self.number_of_subplots = len(labels)  # Number of subplots
        self.y = []  # Storage for the signals in each subplot

        for label in labels:
            if isinstance(label, list):
                self.y.append(np.zeros((len(label), max_length)))  # Multiple signals in the same subplot
            else:
                self.y.append(np.zeros((1, max_length)))  # Single signal in a subplot

        self.t = np.zeros(max_length)
        self.select = select
        self.output_dir_path = output_dir_path
        self.pallet = ['c', 'r', 'g', 'b', 'm', '#E67E22', '#1F618D']
        self.cnt = 0
        self.limits = [[-30,30],[-2,30],[-5,5],[-1.1,1.1],[[-5,5],[-5,5]]]
        
        if not os.path.exists(output_dir_path):
            os.makedirs(output_dir_path)
    
    def record_state(self, t, y):
        self.t[self.cnt] = t
        start_idx = 0

        for i, label in enumerate(self.labels):
            if isinstance(label, list):
                num_signals = len(label)
                self.y[i][:, self.cnt] = np.copy(y[start_idx:start_idx + num_signals])
                start_idx += num_signals
            else:
                self.y[i][0, self.cnt] = y[start_idx]
                start_idx += 1
        
        self.cnt += 1

    def graph(self, j):
        # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>PLOT<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        if self.select['states']:
            fig, axes = plt.subplots(self.number_of_subplots, 1, figsize=(15, 20))
            fig.tight_layout(pad=0.5)

            for i, label in enumerate(self.labels):
                if isinstance(label, list):
                    # Plot multiple signals in one subplot
                    for ii, sublabel in enumerate(label):
                        axes[i].plot(self.t[:self.cnt], self.y[i][ii, :self.cnt], self.pallet[ii % len(self.pallet)], label=sublabel)
                    axes[i].legend(loc='upper right')
                    axes[i].set_ylim(self.limits[i][ii][0],self.limits[i][ii][1])
                else:
                    # Plot single signal in a subplot
                    axes[i].plot(self.t[:self.cnt], self.y[i][0, :self.cnt], self.pallet[0], label=label)
                    axes[i].set_ylim(self.limits[i][0],self.limits[i][1])
                
                axes[i].set_xlabel('t (sec)')
                axes[i].set_ylabel(', '.join(label) if isinstance(label, list) else label)
                axes[i].grid(True)

            plt.grid(color='k', linestyle=':', linewidth=1)
            plt.savefig(self.output_dir_path + '/fig_states_control{}.pdf'.format(j), format='pdf')











class SimResults_():
    def __init__(self, labels, max_length=10000, output_dir_path="./Simulation_Results", select={'states': 1}):
        self.max_length = max_length
        self.number = 5
        self.t = np.zeros(max_length)
        self.y = np.zeros((self.number,max_length))
        self.labels = labels
        self.select = select
        self.output_dir_path = output_dir_path
        self.pallet = ['c','r', 'g', 'b', 'm', '#E67E22', '#1F618D']
        self.cnt = 0 
        
        if not os.path.exists(output_dir_path):
            os.makedirs(output_dir_path)
        
    def record_state(self, t, y):
        self.y[:, self.cnt] = np.copy(y)
        self.t[self.cnt ] = t
        self.cnt +=1
    
    def graph(self, j):
        if self.select['states']:
            fig, axes = plt.subplots(self.number_of_subplots, 1, figsize=(10, 6))
            fig.tight_layout(pad=3.0)

            for i, sublist in enumerate(self.labels):
                for ii, label in enumerate(sublist):
                    axes[i].plot(self.t[:self.cnt], self.y[i][ii, :self.cnt], self.pallet[ii % len(self.pallet)], label=label)
                axes[i].set_xlabel('t (sec)')
                axes[i].legend(loc='upper right')
                axes[i].grid(True)

            plt.grid(color='k', linestyle=':', linewidth=1)
            plt.savefig(self.output_dir_path +
                        '/fig_states_control{}.pdf'.format(j), format='pdf')
            # plt.close(fig1)
            # plt.show()



# class SimResults_():
#     def __init__(self, t, Ctrl, Model, output_dir_path, select={'states': 1}):
#         self.t = t
#         len_t = len(t)
#         self.Model = Model
#         self.Ctrl = Ctrl
#         self.x_s_history = np.zeros((self.dim_n, len_t))
#         self.u_history = np.zeros((self.dim_m, len_t))

#         # the residule of x, attack time
#         self.residule = np.zeros(len_t)
#         self.attack_time = -1

#         self.select = select
#         self.output_dir_path = output_dir_path
#         self.pallet = ['r', 'g', 'b', 'm', '#E67E22', '#1F618D']


#     def record_state(self, i, x, u, x_ref):
#         temp = np.copy(x)
#         temp = temp - x_ref
#         self.x_s_history[:, i] = temp
#         self.u_history[:, i] = u

#     def graph(self, j, i):
#         # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>PLOT<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#         # plot the 'control' + 'states' of the system vs. 'time' ################################
#         if self.select['states']:
#             fig1 = plt.figure()
#             for im in range(self.dim_m):
#                 plt.plot(self.t[:i], self.u_history[im, :i], 'c')
#             for ii in range(self.dim_n):
#                 plt.plot(self.t[:i], self.x_s_history[ii, :i],
#                          self.pallet[ii % len(self.pallet)])

#             plt.legend(
#                 ["Control", "Position", "Velocity", "Accelration"], loc=1)
#             plt.xlabel('t (sec)')
#             plt.ylabel('States and Control')
#             # plt.tight_layout()
#             # plt.ylim((-5, 5))

#             plt.grid(color='k', linestyle=':', linewidth=1)
#             plt.savefig(self.output_dir_path +
#                         '/fig_states_control{}.pdf'.format(j), format='pdf')
#             # plt.close(fig1)
#             # plt.show()

#     # record everything in the movie
#     def record_system(self, i, x, u, x_ref, residule, attack_time=-1):
#         if attack_time != -1:
#             self.attack_time = attack_time
#         temp = np.copy(x)
#         temp = temp - x_ref
#         self.x_s_history[:, i] = temp
#         self.u_history[:, i] = u
#         self.residule[i] = residule

#     def graph_system(self, j, i):
#         # initialize
#         fig, ax = plt.subplots()

#         # plot the control and the system
#         # plot state variable and input
#         for im in range(self.dim_m):
#             ax.plot(self.t[:i], self.u_history[im, :i], 'c')
#         for ii in range(self.dim_n):
#             ax.plot(self.t[:i], self.x_s_history[ii, :i],
#                     self.pallet[ii % len(self.pallet)])

#         fig.legend(
#             ["Control", "Position", "Velocity", "Accelration"], loc=1)
#         plt.xlabel('t (sec)')
#         plt.ylabel('States and Control')

#         if self.attack_time > 0:
#             plt.axvline(x=self.attack_time, color='r', linewidth=0.15)
#             plt.text(self.attack_time, 0, s="attack launched", color='r')

#         plt.grid(color='k', linestyle=':', linewidth=1)
#         plt.savefig(self.output_dir_path +
#                     '/fig_system{}.pdf'.format(j), format='pdf')

#     def graph_system(self, j, i):
#         # initialize
#         fig, ax = plt.subplots()

#         # plot the control and the system
#         # plot state variable and input
#         for im in range(self.dim_m):
#             ax.plot(self.t[:i], self.u_history[im, :i], 'c')
#         for ii in range(self.dim_n):
#             ax.plot(self.t[:i], self.x_s_history[ii, :i],
#                     self.pallet[ii % len(self.pallet)])

#         fig.legend(
#             ["Control", "Position", "Velocity", "Accelration"], loc=1)
#         plt.xlabel('t (sec)')
#         plt.ylabel('States and Control')

#         if self.attack_time > 0:
#             plt.axvline(x=self.attack_time, color='r', linewidth=0.15)
#             plt.text(self.attack_time, -0.05, "attack launched",
#                      transform=ax.get_xaxis_transform(),
#                      ha='center', va='top', color='r')

#         plt.grid(color='k', linestyle=':', linewidth=1)
#         plt.savefig(self.output_dir_path +
#                     '/fig_system{}.pdf'.format(j), format='pdf')

#     def graph_residule(self, j, i):
#         # initialize
#         fig, ax = plt.subplots()

#         # plot the control and the system
#         # plot state variable and input
#         ax.plot(self.t, self.residule, 'c')

#         fig.legend(["residule"], loc=1)
#         plt.xlabel('t (sec)')
#         plt.ylabel('residule value')

#         if self.attack_time > 0:
#             plt.axvline(x=self.attack_time, color='r', linewidth=0.15)
#             # plt.text(self.attack_time, 0, s="attack launched", color='r')
#             plt.text(self.attack_time, -0.05, "attack launched",
#                      transform=ax.get_xaxis_transform(),
#                      ha='center', va='top', color='r')
#         plt.grid(color='k', linestyle=':', linewidth=1)
#         plt.savefig(self.output_dir_path +
#                     '/fig_residule{}.pdf'.format(j), format='pdf')

#     def printout(self, j):
#         # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>PRINT<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#         pass