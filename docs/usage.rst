Usage Guide
===========

This guide explains how to use the `Py-PID` package.

Basic Example
-------------

.. code-block:: python

    from py_pid import PID

    pid = PID(Kp=1.0, Ki=0.1, Kd=0.05, setpoint=10)
    output = pid.run(process_variable=8)

    print(output)

Advanced Features
-----------------

- Derivative-on-measurement
- Output deadbands
- Anti-windup
- Slew rate limiting
- Gain scheduling
