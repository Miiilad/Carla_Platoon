import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize

import gurobipy as gp
from gurobipy import GRB
import os
import matplotlib.gridspec as gridspec
import torch

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
        # self.A = np.array([[0, 1, -1.6],
        #               [0, 0, -1],
        #               [0, 0, -2.17391304]])
        # self.B = np.array([[0],
        #               [0],
        #               [1.59130435]])

        self.A = np.array([[0, 1, -1.6],
                      [0, 0, -1],
                      [0, 0, -2.17391304]])
        self.B = np.array([[0],
                      [0],
                      [1.59130435]])
        self.H=np.array([[0],
                    [1],
                    [0]])
        self.Ad = self.A * h + np.eye(3)
        self.Bd = self.B * h
        self.Hd= self.H * h 

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
        
        Ad = self.A * self.h + np.eye(3)
        Bd = self.B * self.h
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
    
    def find_closest_sample(self,x_k, X_s):
        """
        Finds the closest sample in X_s to x_k using the Euclidean norm.
        
        Parameters:
        x_k : array-like or tensor
            A single input sample.
        X_s : tensor
            A set of input samples (tensor where each row is a sample).
            
        Returns:
        closest_sample : array
            The sample in X_s that is closest to x_k.
        """
        # Convert tensor inputs to numpy arrays
        if isinstance(x_k, torch.Tensor):
            x_k = x_k.numpy()
        
        if isinstance(X_s, torch.Tensor):
            X_s = X_s.numpy()
        
        # Compute Euclidean distance between x_k and each sample in X_s
        distances = np.linalg.norm(X_s - x_k, axis=1)
        
        # Find the index of the minimum distance
        closest_idx = np.argmin(distances)
        
        # Return the closest sample
        return closest_idx
    
    def solve_BF_for_u(self,u,u_lim, x_k, x_s_next, x_s, u_s, a_k, delta_v_k, delta_p_k, a_k_i_minus_1,  Delta_a_k_i_minus_1):
        """
        Solves for u_k given input vectors x_k, x_s and scalar values using Gurobi, handling abs() using auxiliary variables.
        
        Parameters are the same as before.
        
        Returns:
        u_k: float
            The optimized scalar value for u_k
        """
        tau = self.h
        k1, k2, k3 = 10, 10, 10
        varrho_g, varrho_f = 0.001, 0.001
        B_d = self.Bd[2,0]
        d_min = 5
        M = 1e6  # Big-M constant (sufficiently large)
        
        if isinstance(x_s, torch.Tensor):
            x_s = x_s.numpy()
        if isinstance(u_s, torch.Tensor):
            u_s = u_s.numpy()
        if isinstance(x_s_next, torch.Tensor):
            x_s_next = x_s_next.numpy()
        # Create a Gurobi model
        model = gp.Model("qp")
        # model.Params.NonConvex = 2
        model.Params.LogToConsole = 0
        model.Params.FeasibilityTol = 1e-3

        # Define the decision variable u_k
        u_k = model.addVar(lb=-GRB.INFINITY, ub=GRB.INFINITY, name="u_k")

        # Auxiliary variable for |u_k - u_s|
        z_k = model.addVar(lb=0.0, name="z_k")

        # New variable to measure the violation of the constraint
        violation = model.addVar(lb=0.0, name="violation")

        # Binary variable to switch between the cases
        b_k = model.addVar(vtype=GRB.BINARY, name="b_k")

        # Define the objective function
        # Minimize the difference between u and u_k, plus a penalty for constraint violation
        penalty_weight = 10  # Adjust this weight to control the penalty
        objective = (u - u_k) * (u - u_k) + penalty_weight * violation
        model.setObjective(objective, GRB.MINIMIZE)
        # Precompute norms and other scalar values outside the constraint
        norm_x_k = np.linalg.norm(x_k)
        norm_x_k_x_s = np.linalg.norm(x_k - x_s)

        # Big-M constraints to enforce z_k = |u_k - u_s|
        # Case 1: If b_k = 1, z_k = u_k - u_s
        model.addConstr(z_k >= (u_k - u_s) , "abs_pos_1")
        model.addConstr(z_k <= (u_k - u_s) + M * (1 - b_k), "abs_pos_2")

        # Case 2: If b_k = 0, z_k = u_s - u_k
        model.addConstr(z_k >= (-u_k + u_s), "abs_neg_1")
        model.addConstr(z_k <= (-u_k + u_s) + M * b_k, "abs_neg_2")

        # Define the constraint equation using Gurobi LinExpr
        lhs = gp.LinExpr()
        lhs.add(B_d * u_k)
        lhs.add(varrho_g * norm_x_k * z_k)  # Involving z_k which is now |u_k - u_s|
        lhs.add(varrho_g * norm_x_k_x_s * abs(u_s[0]))  # abs(u_s) because it's a scalar
        lhs.add(varrho_f * norm_x_k_x_s)
        lhs.add(-B_d * u_s[0])
        lhs.add(x_s_next[0])
        lhs.add(-a_k)
        lhs.add(Delta_a_k_i_minus_1)
        lhs.add((k1 + k2 + k3) * (a_k - a_k_i_minus_1))
        lhs.add(- (k3 * k1 + k3 * k2 + k2 * k1) / (tau**-1) * delta_v_k)
        lhs.add(- k3 * k2 * k1 / (tau**-2) * (delta_p_k - d_min))

        # Add the constraint to the model
        model.addConstr(lhs - violation <= 0.0, "constraint")
        
        # Add a constraint to limit the violation to a certain threshold
        max_violation = 100  # Set a maximum allowable violation
        model.addConstr(violation <= max_violation, "violation_limit")
        
        # Solve the model


        try:
            # Solve the model
            model.optimize()
            # print(u,u_k.X,u_s)
            # Perform the operations and print each step
            print('u_k:',u_k.X,'    u:', u,'    u_s',u_s, '     z_k:',z_k.X,'    violation:',violation.X)
            lhs_sum = 0
            B_d_u_k = B_d * u_k.X
            print("B_d * u_k:", B_d_u_k)
            lhs_sum += B_d_u_k

            varrho_g_norm_x_k_z_k = varrho_g * norm_x_k * z_k.X  # z_k = |u_k - u_s|
            print("varrho_g * norm_x_k * z_k:", varrho_g_norm_x_k_z_k)
            lhs_sum += varrho_g_norm_x_k_z_k

            varrho_g_norm_x_k_x_s_abs_u_s = varrho_g * norm_x_k_x_s * abs(u_s[0])  # abs(u_s) since u_s is scalar
            print("varrho_g * norm_x_k_x_s * abs(u_s[0]):", varrho_g_norm_x_k_x_s_abs_u_s)
            lhs_sum += varrho_g_norm_x_k_x_s_abs_u_s

            varrho_f_norm_x_k_x_s = varrho_f * norm_x_k_x_s
            print("varrho_f * norm_x_k_x_s:", varrho_f_norm_x_k_x_s)
            lhs_sum += varrho_f_norm_x_k_x_s

            neg_B_d_u_s = -B_d * u_s[0]
            print("-B_d * u_s[0]:", neg_B_d_u_s)
            lhs_sum += neg_B_d_u_s

            x_s_next_value = x_s_next[0]
            print("x_s_next:", x_s_next_value)
            lhs_sum += x_s_next_value

            neg_a_k = -a_k
            print("-a_k:", neg_a_k)
            lhs_sum += neg_a_k

            Delta_a_k_i_minus_1_value = Delta_a_k_i_minus_1
            print("Delta_a_k_i_minus_1:", Delta_a_k_i_minus_1_value)
            lhs_sum += Delta_a_k_i_minus_1_value

            k_terms_a_diff = (k1 + k2 + k3) * (a_k - a_k_i_minus_1)
            print("(k1 + k2 + k3) * (a_k - a_k_i_minus_1):", k_terms_a_diff)
            lhs_sum += k_terms_a_diff

            complex_k_term = - (k3 * k1 + k3 * k2 + k2 * k1) / (tau**-1) * delta_v_k
            print("-(k3 * k1 + k3 * k2 + k2 * k1) / (tau^-1) * delta_v_k:", complex_k_term)
            lhs_sum += complex_k_term

            more_complex_k_term = - k3 * k2 * k1 / (tau**-2) * (delta_p_k - d_min)
            print("- k3 * k2 * k1 / (tau^-2) * (delta_p_k - d_min):", more_complex_k_term)
            lhs_sum += more_complex_k_term

            # Print the total sum
            print("\nTotal sum of all terms:", lhs_sum, lhs_sum-violation.X)
            print('****************\n')

            # Get the optimal value of u_k
            optimal_u_k = u_k.X
            # print(f"Unsafe control {u}, Optimal value of U safe: {optimal_u_k}",violation.X)
            out = np.clip(optimal_u_k, u_lim[0], u_lim[1])
        except gp.GurobiError as e:
            print(f"Gurobi error: {e}")
            out = u
        except Exception as e:
            print(f"General error: {e}")
            out = u

        return out
    
    
    def resilient_solve_BF_for_u(self,u,u_lim, x_k, x_s_next, x_s, u_s, a_k, delta_v_k, delta_p_k, a_k_gamma_i_minus_1,  Delta_a_k_i_minus_1,gamma):
        """
        Solves for u_k given input vectors x_k, x_s and scalar values using Gurobi, handling abs() using auxiliary variables.
        
        Parameters are the same as before.
        
        Returns:
        u_k: float
            The optimized scalar value for u_k
        """
        tau = self.h
        k1, k2, k3 = 10, 10, 10
        varrho_g, varrho_f = 0.001, 0.001
        B_d = self.Bd[2,0]
        d_min = 5
        M = 1e6  # Big-M constant (sufficiently large)
        
        if isinstance(x_s, torch.Tensor):
            x_s = x_s.numpy()
        if isinstance(u_s, torch.Tensor):
            u_s = u_s.numpy()
        if isinstance(x_s_next, torch.Tensor):
            x_s_next = x_s_next.numpy()
        # Create a Gurobi model
        model = gp.Model("qp")
        # model.Params.NonConvex = 2
        model.Params.LogToConsole = 0
        model.Params.FeasibilityTol = 1e-3

        # Define the decision variable u_k
        u_k = model.addVar(lb=-GRB.INFINITY, ub=GRB.INFINITY, name="u_k")

        # Auxiliary variable for |u_k - u_s|
        z_k = model.addVar(lb=0.0, name="z_k")

        # New variable to measure the violation of the constraint
        violation = model.addVar(lb=0.0, name="violation")

        # Binary variable to switch between the cases
        b_k = model.addVar(vtype=GRB.BINARY, name="b_k")

        # Define the objective function
        # Minimize the difference between u and u_k, plus a penalty for constraint violation
        penalty_weight = 10  # Adjust this weight to control the penalty
        objective = (u - u_k) * (u - u_k) + penalty_weight * violation
        model.setObjective(objective, GRB.MINIMIZE)
        # Precompute norms and other scalar values outside the constraint
        norm_x_k = np.linalg.norm(x_k)
        norm_x_k_x_s = np.linalg.norm(x_k - x_s)

        # Big-M constraints to enforce z_k = |u_k - u_s|
        # Case 1: If b_k = 1, z_k = u_k - u_s
        model.addConstr(z_k >= (u_k - u_s) , "abs_pos_1")
        model.addConstr(z_k <= (u_k - u_s) + M * (1 - b_k), "abs_pos_2")

        # Case 2: If b_k = 0, z_k = u_s - u_k
        model.addConstr(z_k >= (-u_k + u_s), "abs_neg_1")
        model.addConstr(z_k <= (-u_k + u_s) + M * b_k, "abs_neg_2")

        # Define the constraint equation using Gurobi LinExpr
        lhs = gp.LinExpr()
        lhs.add(B_d * u_k)
        lhs.add(varrho_g * norm_x_k * z_k)  # Involving z_k which is now |u_k - u_s|
        lhs.add(varrho_g * norm_x_k_x_s * abs(u_s[0]))  # abs(u_s) because it's a scalar
        lhs.add(varrho_f * norm_x_k_x_s)
        lhs.add(-B_d * u_s[0])
        lhs.add(x_s_next[0])
        lhs.add((k1 + k2 + k3-1)*a_k)
        lhs.add(Delta_a_k_i_minus_1)
        lhs.add((k1 + k2 + k3) * (gamma*Delta_a_k_i_minus_1 - a_k_gamma_i_minus_1))
        lhs.add(- (k3 * k1 + k3 * k2 + k2 * k1) / (tau**-1) * delta_v_k)
        lhs.add(- k3 * k2 * k1 / (tau**-2) * (delta_p_k - d_min))

        # Add the constraint to the model
        model.addConstr(lhs - violation <= 0.0, "constraint")
        
        # Add a constraint to limit the violation to a certain threshold
        max_violation = 100  # Set a maximum allowable violation
        model.addConstr(violation <= max_violation, "violation_limit")
        
        # Solve the model


        try:
            # Solve the model
            model.optimize()
            # print(u,u_k.X,u_s)
            # Perform the operations and print each step
            print('u_k:',u_k.X,'    u:', u,'    u_s',u_s, '     z_k:',z_k.X,'    violation:',violation.X)
            lhs_sum = 0
            B_d_u_k = B_d * u_k.X
            print("B_d * u_k:", B_d_u_k)
            lhs_sum += B_d_u_k

            varrho_g_norm_x_k_z_k = varrho_g * norm_x_k * z_k.X  # z_k = |u_k - u_s|
            print("varrho_g * norm_x_k * z_k:", varrho_g_norm_x_k_z_k)
            lhs_sum += varrho_g_norm_x_k_z_k

            varrho_g_norm_x_k_x_s_abs_u_s = varrho_g * norm_x_k_x_s * abs(u_s[0])  # abs(u_s) since u_s is scalar
            print("varrho_g * norm_x_k_x_s * abs(u_s[0]):", varrho_g_norm_x_k_x_s_abs_u_s)
            lhs_sum += varrho_g_norm_x_k_x_s_abs_u_s

            varrho_f_norm_x_k_x_s = varrho_f * norm_x_k_x_s
            print("varrho_f * norm_x_k_x_s:", varrho_f_norm_x_k_x_s)
            lhs_sum += varrho_f_norm_x_k_x_s

            neg_B_d_u_s = -B_d * u_s[0]
            print("-B_d * u_s[0]:", neg_B_d_u_s)
            lhs_sum += neg_B_d_u_s

            x_s_next_value = x_s_next[0]
            print("x_s_next:", x_s_next_value)
            lhs_sum += x_s_next_value

            k_terms_a_k = (k1 + k2 + k3 - 1) * a_k
            print("(k1 + k2 + k3 - 1) * a_k:", k_terms_a_k)
            lhs_sum += k_terms_a_k

            Delta_a_k_i_minus_1_value = Delta_a_k_i_minus_1
            print("Delta_a_k_i_minus_1 (repeated):", Delta_a_k_i_minus_1_value)
            lhs_sum += Delta_a_k_i_minus_1_value

            gamma_term = (k1 + k2 + k3) * (gamma * Delta_a_k_i_minus_1 - a_k_gamma_i_minus_1)
            print("(k1 + k2 + k3) * (gamma * Delta_a_k_i_minus_1 - a_k_gamma_i_minus_1):", gamma_term)
            lhs_sum += gamma_term

            complex_k_term_2 = - (k3 * k1 + k3 * k2 + k2 * k1) / (tau**-1) * delta_v_k
            print("-(k3 * k1 + k3 * k2 + k2 * k1) / (tau^-1) * delta_v_k (repeated):", complex_k_term_2)
            lhs_sum += complex_k_term_2

            more_complex_k_term_2 = - k3 * k2 * k1 / (tau**-2) * (delta_p_k - d_min)
            print("- k3 * k2 * k1 / (tau^-2) * (delta_p_k - d_min) (repeated):", more_complex_k_term_2)
            lhs_sum += more_complex_k_term_2

            # Print the total sum
            print("\nTotal sum of all terms:", lhs_sum, lhs_sum-violation.X)
            print('****************\n')

            # Get the optimal value of u_k
            optimal_u_k = u_k.X
            # print(f"Unsafe control {u}, Optimal value of U safe: {optimal_u_k}",violation.X)
            out = np.clip(optimal_u_k, u_lim[0], u_lim[1])
        except gp.GurobiError as e:
            print(f"Gurobi error: {e}")
            out = u
        except Exception as e:
            print(f"General error: {e}")
            out = u

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
    def __init__(self, labels,limits,linestyles, max_length=10000, output_dir_path="./Simulation_Results", select={'states': 1}):
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
        self.pallet = ['c', 'r--', 'g--', 'b', 'm', '#E67E22', '#1F618D']
        self.pallet = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf']
        self.cnt = 0
        self.limits = limits
        self.linestyles = linestyles
        
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
            fig.tight_layout(pad=2)

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
            
    def graph_(self, j):
    # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>PLOT<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        if self.select['states']:
            # Create a grid layout with flexible heights
            heights = [2 if isinstance(label, list) else 1 for label in self.labels]  # Give more height to subplots with multiple labels
            fig = plt.figure(figsize=(15, 20))
            gs = gridspec.GridSpec(self.number_of_subplots, 1, height_ratios=heights)
            
            axes = [fig.add_subplot(gs[i]) for i in range(self.number_of_subplots)]
            fig.tight_layout(pad=2)

            for i, label in enumerate(self.labels):
                if isinstance(label, list):
                    # Plot multiple signals in one subplot
                    for ii, sublabel in enumerate(label):
                        axes[i].plot(self.t[:self.cnt], self.y[i][ii, :self.cnt], self.pallet[ii % len(self.pallet)],linestyle=self.linestyles[i][ii], label=sublabel)
                    axes[i].legend(loc='upper right')
                    axes[i].set_ylim(self.limits[i][ii][0], self.limits[i][ii][1])
                else:
                    # Plot single signal in a subplot
                    axes[i].plot(self.t[:self.cnt], self.y[i][0, :self.cnt], self.pallet[0],linestyle=self.linestyles[i], label=label)
                    axes[i].set_ylim(self.limits[i][0], self.limits[i][1])

                axes[i].set_xlabel('t (sec)')
                axes[i].set_ylabel(', '.join(label) if isinstance(label, list) else label)
                axes[i].grid(True)

            plt.grid(color='k', linestyle=':', linewidth=1)
            plt.savefig(self.output_dir_path + '/fig_states_control{}.pdf'.format(j), format='pdf')




