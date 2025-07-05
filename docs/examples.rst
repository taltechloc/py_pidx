Examples
========

This section contains practical usage examples to help you get started with `py_pidx`.

Basic PID Control
-----------------

This example demonstrates a simple PID control loop with fixed gains. It is the foundation of most PID applications where proportional, integral, and derivative terms are tuned manually.

Why use this?
- Simple to implement and understand.
- Suitable for many stable systems with consistent behavior.

Pros:
- Easy tuning.
- Low computational overhead.

Cons:
- Fixed gains may not perform well under varying conditions or nonlinear systems.

.. code-block:: python

    from py_pidx import PID
    import time

    pid = PID(Kp=1.0, Ki=0.1, Kd=0.05, setpoint=20)

    for _ in range(10):
        pv = 15  # Example process variable
        output = pid.run(pv)
        print(f"Control output: {output}")
        time.sleep(0.1)

Gain Scheduling Example
-----------------------

Gain scheduling dynamically adjusts the PID gains based on the current process variable (PV). This allows the controller to better handle nonlinearities or different operating regimes.

Why use this?
- Improves performance when system dynamics change with PV.
- Avoids manual retuning for different conditions.

Pros:
- More adaptive control.
- Smooth transition between gain sets.

Cons:
- Requires knowledge of appropriate gain values for each range.
- More complex to implement and tune.

.. code-block:: python

    from py_pidx import PID
    from py_pidx.gain_scheduler import GainScheduler

    class MyScheduler(GainScheduler):
        def get_gains(self, pv):
            if pv < 5:
                return (2.0, 0.1, 0.01)
            elif pv < 10:
                return (1.5, 0.05, 0.005)
            else:
                return (1.0, 0.01, 0.001)

    pid = PID(gain_scheduler=MyScheduler())

    pv = 7
    output = pid.run(pv)
    print(f"Output with gain scheduling: {output}")

Deadband and Anti-windup Example
--------------------------------

Deadbands help to ignore small errors or output changes that might cause actuator jitter. Anti-windup prevents integral term buildup when actuators saturate.

Why use this?
- Reduces unnecessary actuator movement.
- Prevents integral overshoot and instability.

Pros:
- Smoother control signals.
- Better handling of saturation limits.

Cons:
- Choosing deadband thresholds can be tricky.
- May reduce control sensitivity if deadbands are too large.

.. code-block:: python

    pid = PID(
        Kp=1.0, Ki=0.2, Kd=0.1,
        error_deadband_limits=(-0.1, 0.1),
        output_deadband_limits=(-0.05, 0.05),
        anti_windup=True
    )

    pv = 9.8
    output = pid.run(pv)
    print(f"Output with deadbands and anti-windup: {output}")

Advanced Control Loop with Feedforward
--------------------------------------

Feedforward allows adding an anticipatory term to improve response, while derivative filtering smooths noisy derivative calculations.

Why use this?
- Improves system response speed.
- Reduces sensitivity to measurement noise.

Pros:
- Better setpoint tracking.
- More stable derivative action.

Cons:
- Feedforward requires model or experience.
- Filter tuning adds complexity.

.. code-block:: python

    pid = PID(
        Kp=1.0, Ki=0.1, Kd=0.05,
        feedforward=2.0,
        derivative_filter=0.2
    )

    pv = 5.0
    output = pid.run(pv)
    print(f"Output with feedforward and filtering: {output}")

Slew Rate Limiting Example
--------------------------

Slew rate limiting restricts how fast the controller output can change between updates, protecting actuators from rapid or harsh movements.

Why use this?
- Prevents actuator wear or shock.
- Improves safety in sensitive systems.

Pros:
- Smooth actuator commands.
- Avoids abrupt changes.

Cons:
- May slow system response.
- Requires tuning of max rate.

.. code-block:: python

    pid = PID(
        Kp=1.0, Ki=0.1, Kd=0.05,
        max_output_rate=1.0  # Limit output rate to 1 unit per run call
    )

    pv = 0
    outputs = []
    for i in range(5):
        output = pid.run(pv)
        outputs.append(output)
        print(f"Output with slew rate limiting: {output}")
        pv += 2  # simulate process variable change

Integral and Derivative on Measurement Example
----------------------------------------------

This option calculates the integral and derivative terms based on the process variable (measurement) rather than the error, which can reduce noise and improve robustness in some scenarios.

Why use this?
- Reduces derivative kick and noise.
- Better for systems with noisy setpoints.

Pros:
- Smoother control output.
- Reduces sensitivity to setpoint changes.

Cons:
- May reduce responsiveness to sudden setpoint changes.
- Not always appropriate depending on control goals.

.. code-block:: python

    pid = PID(
        Kp=1.0, Ki=0.2, Kd=0.1,
        integral_on_measurement=True,
        derivative_on_measurement=True
    )

    pv = 10
    output = pid.run(pv)
    print(f"Output with integral and derivative on measurement: {output}")

Resetting and Toggling Auto Mode Example
----------------------------------------

This example shows how to pause and resume the PID controller, as well as how to reset its internal state. Useful for testing, system startup, or manual override.

Why use this?
- Allows manual control or safe startup.
- Clears internal integrator and derivative memory.

Pros:
- Gives control over PID lifecycle.
- Prevents control windup on pause.

Cons:
- Requires careful handling to avoid control gaps.

.. code-block:: python

    pid = PID(Kp=1.0, Ki=0.1, Kd=0.05)
    pid.set_auto_mode(False)  # Pause controller
    print("Controller paused")

    pid.set_auto_mode(True)   # Resume controller
    print("Controller resumed")

    pid.reset()               # Clear internal states
    print("Controller reset")
