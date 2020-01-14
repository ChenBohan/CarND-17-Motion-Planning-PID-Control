# CarND-18-Motion-Planning-PID-Control
Udacity Self-Driving Car Engineer Nanodegree: PID Control.

## Overview

<img src="https://github.com/ChenBohan/CarND-17-Motion-Planning-PID-Control/blob/master/readme_img/overview.png" width = "100%" height = "100%" div align=center />

- How do we generate input?

- How do we drive error to zero over time?

## P control

- P = proportional

- Using present infomation.

- `cte`: cross track error.

- `tau`: response strength of the proportional controller.

- Your steering angle, alpha, equals a proportional factor of `tau` to the cross track error. What will happen to the car?
    - Car overshoots.
```
('y: ', 0.2823092286467954, ' steer: ', -0.03506168379843757)
('y: ', 0.2124225375861215, ' steer: ', -0.028230922864679542)
('y: ', 0.14130182577480355, ' steer: ', -0.02124225375861215)
('y: ', 0.06965132942895698, ' steer: ', -0.014130182577480355)
('y: ', -0.002703889136699514, ' steer: ', -0.006965132942895698)
('y: ', -0.07540645276524681, ' steer: ', 0.00027038891366995137)
('y: ', -0.14809553271809206, ' steer: ', 0.007540645276524681)
('y: ', -0.22040856550422522, ' steer: ', 0.014809553271809207)
('y: ', -0.2914332738077974, ' steer: ', 0.022040856550422525)
```

- Tau, the control parameter, changes from 0.1 to 0.3. What happens?
    - Oscillates faster.

```python
def run(robot, tau, n=100, speed=1.0):
    x_trajectory = []
    y_trajectory = []
    for i in range(n):
        cte = robot.y
        steer = -tau * cte
        robot.move(steer, speed)
        x_trajectory.append(robot.x)
        y_trajectory.append(robot.y)
    return x_trajectory, y_trajectory
```

pic

Disadvantage of P-control: error won't go zero, just get smaller -> **steady state error**.

## PD control

- Using present + past infomation.

We've added the `prev_cte` variable which is assigned to the previous CTE and `diff_cte`, the difference between the current CTE and previous CTE.

```python
def run(robot, tau_p, tau_d, n=100, speed=1.0):
    x_trajectory = []
    y_trajectory = []
    prev_cte = robot.y
    for i in range(n):
        cte = robot.y
        diff_cte = cte - prev_cte
        prev_cte = cte
        steer = -tau_p * cte - tau_d * diff_cte
        robot.move(steer, speed)
        x_trajectory.append(robot.x)
        y_trajectory.append(robot.y)
    return x_trajectory, y_trajectory
```

## PID control

With the integral term we're keeping track of all the previous CTEs, initially we set `int_cte` to 0 and then add the current `cte` term to the count `int_cte += cte`.

```python
def run(robot, tau_p, tau_d, tau_i, n=100, speed=1.0):
    x_trajectory = []
    y_trajectory = []
    prev_cte = robot.y
    int_cte = 0
    for i in range(n):
        cte = robot.y
        diff_cte = cte - prev_cte
        prev_cte = cte
        int_cte += cte
        steer = -tau_p * cte - tau_d * diff_cte - tau_i * int_cte
        robot.move(steer, speed)
        x_trajectory.append(robot.x)
        y_trajectory.append(robot.y)
    return x_trajectory, y_trajectory
```

## Parameter optimization

```python
def twiddle(tol=0.2): 
    p = [0, 0, 0]
    dp = [1, 1, 1]
    robot = make_robot()
    x_trajectory, y_trajectory, best_err = run(robot, p)

    it = 0
    while sum(dp) > tol:
        print("Iteration {}, best error = {}".format(it, best_err))
        for i in range(len(p)):
            p[i] += dp[i]
            robot = make_robot()
            x_trajectory, y_trajectory, err = run(robot, p)

            if err < best_err:
                best_err = err
                dp[i] *= 1.1
            else:
                p[i] -= 2 * dp[i]
                robot = make_robot()
                x_trajectory, y_trajectory, err = run(robot, p)

                if err < best_err:
                    best_err = err
                    dp[i] *= 1.1
                else:
                    p[i] += dp[i]
                    dp[i] *= 0.9
        it += 1
    return p
```

Also, with twiddle the PID controller converges faster but we overshoot drastically at first so it's a tradeoff.

<img src="https://github.com/ChenBohan/CarND-17-Motion-Planning-PID-Control/blob/master/readme_img/performance.png" width = "50%" height = "50%" div align=center />
