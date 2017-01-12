import numpy as np


class controller:
    """ Implements an operational space controller (OSC)
    """

    def __init__(self, robot_config, kp=100, kv=None, vmax=0.5):

        self.robot_config = robot_config

        # proportional gain term
        self.kp = kp
        # derivative gain term
        self.kv = np.sqrt(self.kp) if kv is None else kv
        # velocity limit of the end-effector
        self.vmax = vmax
        self.lamb = self.kp / self.kv

    def control(self, q, dq,
                target_x, target_dx=np.zeros(6),
                x=[0, 0, 0], ee_name='EE',
                mask=[1, 1, 1, 0, 0, 0]):
        """ Generates the control signal

        q np.array: the current joint angles
        dq np.array: the current joint velocities
        target_x np.array: the target end-effector position and orientation
        target_dx np.array: the target end-effector velocity
        x list: offset of the point of interest from the frame of reference
        ee_name string: name of end-effector to control, passed into config
        mask list: indicates the [x, y, z, roll, pitch, yaw] dimensions
                   to be controlled
        """
        # TODO: rework so only have to provide as many target values
        # as there are 1s in the mask

        # the number of dimensions controlled
        dim = np.sum(mask)

        # calculate the end-effector position information
        xyz = self.robot_config.Tx(ee_name, q, x=x)
        orientation = self.robot_config.orientation(ee_name, q)
        x = np.hstack([xyz, orientation])
        # apply mask to isolate position variables of interest
        x = [x[ii] for ii in range(6) if mask[ii] == 1]
        print('x: ', x)

        # calculate the Jacobian for the end effector
        JEE = self.robot_config.J(ee_name, q, x=x)
        # apply mask to isolate DOF of interest
        JEE = np.array([JEE[ii] for ii in range(6) if mask[ii] == 1])

        # calculate the end-effector orientation information
        dx = np.dot(JEE, dq)
        print('dx: ', dx)

        # calculate the inertia matrix in joint space
        Mq = self.robot_config.Mq(q)

        # calculate the effect of gravity in joint space
        Mq_g = self.robot_config.Mq_g(q)

        Mq_inv = np.linalg.inv(Mq)
        JEE_Mq_inv = np.dot(JEE, Mq_inv)

        # convert the mass compensation into end effector space
        Mx_inv = np.dot(JEE_Mq_inv, JEE.T)
        # using the rcond to set singular values < thresh to 0
        # is slightly faster than doing it manually with svd
        Mx = np.linalg.pinv(Mx_inv, rcond=.01)

        # calculate desired force in (x,y,z) space
        if self.vmax is not None:
            # implement velocity limiting
            x_tilde = x - target_x
            sat = self.vmax / (self.lamb * np.abs(x_tilde))
            if np.any(sat < 1):
                index = np.argmin(sat)
                unclipped = self.kp * x_tilde[index]
                clipped = self.kv * self.vmax * np.sign(x_tilde[index])
                scale = np.ones(3, dtype='float32') * clipped / unclipped
                scale[index] = 1
            else:
                scale = np.ones(3, dtype='float32')

            u_xyz = -self.kv * (dx - target_dx -
                                np.clip(sat / scale, 0, 1) *
                                -self.lamb * scale * x_tilde)
        else:
            # generate (x,y,z) force without velocity limiting)
            u_xyz = (self.kp * (target_x - x) +
                     self.kv * (target_dx - dx))
        print('u_xyz: ', u_xyz)

        u_xyz = np.dot(Mx, u_xyz)

        # TODO: This is really awkward, but how else to get out
        # this signal for dynamics adaptation training?
        self.training_signal = np.dot(JEE.T, u_xyz)
        # add in gravity compensation, not included in training signal
        u = self.training_signal - Mq_g

        # NOTE: Should the null space controller be separated out
        # as a signal to be added in if chosen? -----------------

        # calculate the null space filter
        Jdyn_inv = np.dot(Mx, JEE_Mq_inv)
        null_filter = (np.eye(self.robot_config.num_joints) -
                       np.dot(JEE.T, Jdyn_inv))

        q_des = np.zeros(self.robot_config.num_joints, dtype='float32')
        dq_des = np.zeros(self.robot_config.num_joints, dtype='float32')

        # calculated desired joint angle acceleration using rest angles
        for ii in range(1, self.robot_config.num_joints):
            if self.robot_config.rest_angles[ii] is not None:
                q_des[ii] = (
                    ((self.robot_config.rest_angles[ii] - q[ii]) + np.pi) %
                     (np.pi*2) - np.pi)
                dq_des[ii] = dq[ii]
        # only compensate for velocity for joints with a control signal
        nkp = self.kp * .1
        nkv = np.sqrt(nkp)
        u_null = np.dot(Mq, (nkp * q_des - nkv * dq_des))

        u += np.dot(null_filter, u_null)

        return u
