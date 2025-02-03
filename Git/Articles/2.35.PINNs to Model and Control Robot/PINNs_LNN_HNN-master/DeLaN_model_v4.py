import jax
import jax.numpy as jnp
import haiku as hk
import numpy as np
from functools import partial

"""
    This file contains definitions of necessary Matrix Initializations and Loss function
    Incorporate with franka_delan.py will help better understand how to implement this
"""
#--------------4th Order Runge - Kutta integration--------------#
def rk4_step(f, x, y, t, h):
    k1 = h * f(x, y, t)
    k2 = h * f(x + k1/2, y, t + h/2)
    k3 = h * f(x + k2/2, y, t + h/2)
    k4 = h * f(x + k3, y, t + h)
    return x + 1/6 * (k1 + 2 * k2 + 2 * k3 + k4)

#----------Compute the Mass matrix (M)-------------------#
def mass_matrix_fn(q, n_dof, shape, activation, epsilon, shift):
    """
    Summary of all abbreviations:
        q     : Input features (angle).
        n_dof : Number of degrees of freedom (in our publication this will be set to 3).
        shape : A tuple defining the architecture of the neural network (number of neurons in each layer).
        activation: Activation function used in the neural network
        epsilon : A small positive value to ensure numerical stability for diagonal entries (as referred in Yellow pg 4/17).
        shift : A value added to the diagonal entries before applying Soft-plus.
    """
    #--The number of values of the lower triangle matrix that needed to be extracted from the NNs--#
    n_output = int((n_dof ** 2 + n_dof) / 2)  

    #--Compute Matrix Indices--#
    net = hk.nets.MLP(output_sizes=shape + (n_output, ), 
                      activation=activation,
                      name="mass_matrix")
    
    """
    jnp.split: Divides the output into two parts:
        l_diagonal: First n_dof values, corresponding to the diagonal entries of the triangular matrix.
        l_off_diagonal: Remaining values, corresponding to the lower triangular off-diagonal entries.
    """
    l_diagonal, l_off_diagonal = jnp.split(net(q), [n_dof, ], axis=-1)

    #--Ensure positive diagonal--#
    l_diagonal = jax.nn.softplus(l_diagonal + shift) + epsilon

    #--Create a n_dof x n_dof matrix of zeros--#
    triangular_mat = jnp.zeros((n_dof, n_dof))
    #--Get the diagonal indices--#
    diagonal_index = np.diag_indices(n_dof)
    #--Get the below-the-diagonal indices--#
    tril_index = np.tril_indices(n_dof, -1)
    #--Compute the L matrix by combining diagonal elements and below-diagonal elements--#
    triangular_mat = triangular_mat.at[diagonal_index].set(l_diagonal[:])
    triangular_mat = triangular_mat.at[tril_index].set(l_off_diagonal[:])

    #--Cholesky decomposition: M = L @ L.T--#
    mass_mat = jnp.matmul(triangular_mat, triangular_mat.transpose())
    return mass_mat

#-------------Compute the Dissipative matrix (D(q))------------#
"""
    According to the publication, the D(q) bears a resemblance to M(q) 
    so we utilize the same NNs
"""
def dissipative_matrix(q, n_dof, shape, activation):
    assert n_dof > 0
    n_output = int((n_dof ** 2 + n_dof) / 2)  # the number of values of the lower triangle matrix

    # Compute Matrix Indices
    net = hk.nets.MLP(output_sizes=shape + (n_output, ),  # shape defined the layers and their neural numbers
                      activation=activation,
                      name="dissipative_matrix")

    # scaler to constraint the matrix
    scaler = 0.4
    l_diagonal, l_off_diagonal = jnp.split(net(q), [n_dof, ], axis=-1)

    l_diagonal = jax.nn.sigmoid(l_diagonal)

    triangular_mat = jnp.zeros((n_dof, n_dof))
    diagonal_index = np.diag_indices(n_dof)
    tril_index = np.tril_indices(n_dof, -1)
    triangular_mat = triangular_mat.at[diagonal_index].set(l_diagonal[:])
    triangular_mat = triangular_mat.at[tril_index].set(l_off_diagonal[:])

    dissipative_mat = jnp.matmul(triangular_mat, triangular_mat.transpose())
    dissipative_mat *= scaler
    return dissipative_mat

#-------------Compute the Transformation matrix (A(q))------------#
def input_transform_matrix(q, n_dof, actuator_dof, shape, activation):
    #--Output size equals the size of 9 which is then reshape into a 3x3 A(q) Matrix--#
    n_output = n_dof * actuator_dof
    net = hk.nets.MLP(output_sizes=shape + (n_output, ),  
                      activation=activation,
                      name="input_transform_matrix")
    #--This one is to reshape--#
    input_mat = net(q).reshape(n_dof, actuator_dof)

    return input_mat

#------------Compute the Kinetic Energy follow the equation: T = 1/2 q_dot.T @ M @ q_dot-----------#
def kinetic_energy_fn(q, qd, n_dof, shape, activation, epsilon, shift):
    mass_mat = mass_matrix_fn(q, n_dof, shape, activation, epsilon, shift)
    return 1./2. * jnp.dot(qd, jnp.dot(mass_mat, qd))

#------------Compute the Potential Energy V via a simple DNNs-------------------#
def potential_energy_fn(q, shape, activation):
    net = hk.nets.MLP(output_sizes=shape +(1, ),
                      activation=activation,
                      name="potential_energy")

    # Apply feature transform
    return net(q)

#------------Compute the Lagrangian function follow the equation: L = T - V ------#
def structured_lagrangian_fn(q, qd, n_dof, shape, activation, epsilon, shift):
    e_kin = kinetic_energy_fn(q, qd, n_dof, shape, activation, epsilon, shift)
    e_pot = potential_energy_fn(q, shape, activation).squeeze()
    return e_kin - e_pot

#------------Perform the forward pass, result is the q_Ddot (angular accelerations)--------------#
def forward_model(params, key, lagrangian, dissipative_mat, input_mat, n_dof):
    """
    Summary of abbreviations:
        params: A dictionary containing parameters required for the Lagrangian, dissipative term, and input transformation.
        lagrangian: A function that computes the Lagrangian of the system structured_lagrangian_fn()
        dissipative_mat: A function that calculates the dissipative term D(q).
        input_mat: A function that computes the input transformation matrix A(q)
    """
    def equation_of_motion(state, tau, t=None):
        #--Split the state into 2 halves: first 3 elements represent angles (q) and the last represent angular velocities (q_dot)--#
        q, qd = jnp.split(state, 2)
        #--argnums: Defines the indices of the arguments in the lagrangian function that will be differentiated.--#
        #--These are typically the generalized coordinates (q) and angular velocities (qd).--#
        argnums = [2, 3]

        #--Extract Lagrangian parameters: Retrieves the Lagrangian parameters from the params dictionary.--#
        l_params = params["lagrangian"]

        #--Compute the value of Gradient of Lagrangian w.r.t angles (q) and angular velocities (q_dot)--# 
        lagrangian_value_and_grad = jax.value_and_grad(lagrangian, argnums=argnums)
        L, (dLdq, dLdqd) = lagrangian_value_and_grad(l_params, key, q, qd)

        #--Compute the value of Hessian of Lagrangian w.r.t angles (q) and angular velocities (q_dot)--#
        lagrangian_hessian = jax.hessian(lagrangian, argnums=argnums)
        (_, (d2L_dqddq, d2Ld2qd)) = lagrangian_hessian(l_params, key, q, qd)

        #--Compute Dissipative term--#
        d_params = params["dissipative"]
        dissipative = dissipative_mat(d_params, key, q)

        #--For A(q) as a net--#
        i_params = params["input_transform"]
        input_transform = input_mat(i_params, key, q)

        #--Compute q_Ddot--#
        qdd_pred = jnp.linalg.pinv(d2Ld2qd) @ \
                   (input_transform @ tau - d2L_dqddq @ qd + dLdq - dissipative @ qd)

        return jnp.concatenate([qd, qdd_pred])
    return equation_of_motion


def inverse_model(params, key, lagrangian, dissipative_mat, input_mat, n_dof):
    def equation_of_motion(state, qdd=None,  t=None):
        # state should be a (n_dof * 3) np.array
        q, qd = jnp.split(state, 2)
        argnums = [2, 3]

        l_params = params["lagrangian"]

        # Compute Lagrangian and Jacobians:
        # def structured_lagrangian_fn(q, qd, n_dof, shape, activation, epsilon, shift):
        lagrangian_value_and_grad = jax.value_and_grad(lagrangian, argnums=argnums)
        L, (dLdq, dLdqd) = lagrangian_value_and_grad(l_params, key, q, qd)

        # Compute Hessian:
        lagrangian_hessian = jax.hessian(lagrangian, argnums=argnums)
        (_, (d2L_dqddq, d2Ld2qd)) = lagrangian_hessian(l_params, key, q, qd)

        # Compute Dissipative term
        d_params = params["dissipative"]
        # def dissipative_matrix(qd, n_dof, shape, activation):
        dissipative = dissipative_mat(d_params, key, q)

        i_params = params["input_transform"]
        input_transform = input_mat(i_params, key, q)

        # Compute the inverse model
        tau = jnp.linalg.inv(input_transform) @ (d2Ld2qd @ qdd + d2L_dqddq @ qd - dLdq + dissipative @ qd)
        return tau
    return equation_of_motion

def loss_fn(params, q, qd, tau, q_next, qd_next, lagrangian, dissipative_mat, input_mat, n_dof, time_step=None):
    """
    Summary of abbreviations:
        q, qd: Recorded data of angles, angular velocities at time t_k
        q_next, qd_next: Recorded data of angles, angular velocities at time t_k+1
        tau: Recorded data of input torques at time t_k
        time_step: Optional time step used for numerical integration (e.g., for Runge-Kutta 4 method).
    """
    states = jnp.concatenate([q, qd], axis=1)
    targets = jnp.concatenate([q_next, qd_next], axis=1)

    #--The forward_model function is JIT (Just-In-Time) compiled using jax.jit for performance optimization.--#
    f = jax.jit(forward_model(params=params, key=None, lagrangian=lagrangian, dissipative_mat=dissipative_mat, input_mat=input_mat, n_dof=n_dof))
    if time_step is not None:
        preds = jax.vmap(partial(rk4_step, f, t=0.0, h=time_step), (0, 0))(states, tau)
    else:
        preds = jax.vmap(f, (0, 0))(states, tau)

    forward_error = jnp.sum((targets - preds)**2, axis=-1)
    mean_forward_error = jnp.mean(forward_error)
    var_forward_error = jnp.var(forward_error)

    # Compute Loss
    loss =  mean_forward_error

    logs = {
        'loss': loss,
        'forward_mean': mean_forward_error,
        'forward_var': var_forward_error,
    }
    return loss, logs
