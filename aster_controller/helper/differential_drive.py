from math import sin, cos, pi

class DifferentialDriveRobot:
    '''
    Class representing the Differential Drive Robot model

    Attributes
    ----------
    wheel_base: float
        The distance between the center of the left and right wheel
    wheel_radius: float
        The radius of the robot wheel
    linear_velocity: float
        The desired linear velocity of the robot
    angular_velocity: float
        The desired angular velocity of the robot

    x: float
        The x coordinate of the robot in metres
    y: float
        The y coordinate of the robot in metres
    theta: float
        The orientation angle of the robot from the x axis in radians

    x_vel: float
        The measured linear velocity of the robot in the x direction
    y_vel: float
        The measured linear velocity of the robot in the y direction
    theta_vel:
        The measured angular velocity of the robot

    Methods
    -------
    compute_wheel_angular_velocities():
        Calculates the desired wheel angular velocities given the robot's
        desired linear and angular velocity

    compute_odometry(left_enc, right_enc, delta_time)
        Calculates the odometry of the robot i.e. the x position, y position
        and the orientation angle theta
    '''

    def __init__(self, wheel_base, wheel_radius):
        self.wheel_base = wheel_base
        self.wheel_radius = wheel_radius
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.x_vel = 0.0
        self.y_vel = 0.0
        self.theta_vel = 0.0

    def compute_wheel_angular_velocities(self):
        '''
        Calculates the desired wheel angular velocities given the robot's
        desired linear and angular velocity

        Parameters
        ----------
        None

        Returns
        -------
        left_wheel_angular_velocity: float
            Angular velocity of the left wheel

        right_wheel_angular_velocity: float
            Angular velocity of the right wheel
        '''
        v = self.linear_velocity
        w = self.angular_velocity
        L = self.wheel_base
        R = self.wheel_radius

        left_wheel_angular_velocity = ((2.0 * v) - (w * L)) / (2.0 * R)
        right_wheel_angular_velocity = ((2.0 * v) + (w * L)) / (2.0 * R)

        return left_wheel_angular_velocity, right_wheel_angular_velocity

    def compute_odometry(self, left_enc, right_enc, delta_time):
        '''
        Calculates the odometry of the robot i.e. the x position,
        y position, and the orientation angle theta

        Parameters
        ----------
        left_enc: Encoder
            The left wheel's encoder sensor object
        right_enc: Encoder
            The right wheel's encoder sensor object
        delta_time: float
            The sampling time interval at which this function is called

        Returns
        -------
        None
        '''
        d_left = (left_enc.direction * left_enc.increment * 2 * pi * self.wheel_radius) / left_enc.ticks_per_rev
        d_right = (right_enc.direction * right_enc.increment * 2 * pi * self.wheel_radius) / right_enc.ticks_per_rev

        delta_dist = (d_left + d_right) / 2
        delta_theta = (d_right - d_left) / self.wheel_base
        delta_x = 0
        delta_y = 0

        if d_left == d_right:
            delta_x = delta_dist * cos(self.theta)
            delta_y = delta_dist * sin(self.theta)
        else:
            radius = delta_dist / delta_theta
            icc_x = self.x - radius * sin(self.theta)
            icc_y = self.y + radius * cos(self.theta)

            delta_x = cos(delta_theta) * (self.x - icc_x) \
                      - sin(delta_theta) * (self.y - icc_y) \
                      + icc_x - self.x

            delta_y = sin(delta_theta) * (self.x - icc_x) \
                      + cos(delta_theta) * (self.y - icc_y) \
                      + icc_y - self.y

        self.x += delta_x
        self.y += delta_y
        self.theta = (self.theta + delta_theta) % (2 * pi)
        self.x_vel = delta_dist / delta_time if delta_time > 0 else 0.0
        self.y_vel = 0.0
        self.theta_vel = delta_theta / delta_time if delta_time > 0 else 0.0
