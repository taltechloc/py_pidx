Introduction
============

py_pidx
-------

**py_pidx** is an advanced Python library for PID control that provides robust, real-world-ready features, including:

- Gain scheduling
- Deadband handling (error & output)
- Anti-windup
- Slew rate limiting
- Derivative filtering
- Feedforward support
- Precision control with dynamic time steps
- Integral and derivative-on-measurement option

Installation
------------

.. code-block:: bash

    pip install py_pidx

Features Overview
-----------------

+---------------------------+------------------------------------------------------------------+
| Feature                   | Description                                                      |
+===========================+==================================================================+
| `Kp`, `Ki`, `Kd`          | Tunable proportional, integral, and derivative gains            |
+---------------------------+------------------------------------------------------------------+
| `GainScheduler`           | Dynamically adjust PID gains at runtime                          |
+---------------------------+------------------------------------------------------------------+
| Deadbands                 | Zero out small control efforts or errors to reduce actuator jitter |
+---------------------------+------------------------------------------------------------------+
| Anti-windup               | Prevents runaway integral values during saturation               |
+---------------------------+------------------------------------------------------------------+
| Derivative on Measurement | Optionally compute D-term from PV instead of error               |
+---------------------------+------------------------------------------------------------------+
| Integral on Measurement   | Optionally compute I-term on process variable instead of error   |
+---------------------------+------------------------------------------------------------------+
| Low-pass Filtering        | Smooths noisy derivative term using alpha filtering              |
+---------------------------+------------------------------------------------------------------+
| Feedforward               | Optional term to improve open-loop response                      |
+---------------------------+------------------------------------------------------------------+
| Slew Rate Limiting        | Prevents rapid output changes that may shock actuators           |
+---------------------------+------------------------------------------------------------------+

Basic Usage
-----------

.. code-block:: python

    from py_pidx import PID

    pid = PID(Kp=1.2, Ki=0.5, Kd=0.05, setpoint=100)

    while True:
        pv = read_sensor()  # Your process variable
        control_signal = pid.run(pv)
        send_to_actuator(control_signal)
        time.sleep(0.01)

.. _advanced-features-intro:

Advanced Features
-----------------

Gain Scheduling
~~~~~~~~~~~~~~~

.. code-block:: python

    from py_pidx.gain_scheduler import GainScheduler

    class MyScheduler(GainScheduler):
        def get_gains(self, pv):
            if pv < 10:
                return (2.0, 0.1, 0.01)
            elif pv < 20:
                return (1.5, 0.05, 0.005)
            else:
                return (1.0, 0.01, 0.001)

    pid = PID(gain_scheduler=MyScheduler())

Deadbands
~~~~~~~~~

.. code-block:: python

    pid = PID(
        output_deadband_limits=(-0.1, 0.1),
        error_deadband_limits=(-0.05, 0.05)
    )

Anti-Windup and Slew Rate
~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

    pid = PID(
        Ki=0.3,
        anti_windup=True,
        max_output_rate=10.0  # Max 10 units/sec change
    )

Derivative Filtering and Feedforward
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

    pid = PID(
        Kd=0.2,
        derivative_filter=0.3,  # Low-pass smoothing factor (0.0–1.0)
        feedforward=1.5
    )

Testing / Resetting / Toggling
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

    pid.set_auto_mode(False)    # Pause control
    pid.set_auto_mode(True)     # Resume control
    pid.reset()                 # Clear internal state

Contact & Support
-----------------

Maintained by **Mehrab Mahdian**.

- Email: memahdian@outlook.com, mehrab.mahdian@taltech.ee
- GitHub: https://github.com/mehrabmahdian
- LinkedIn: https://www.linkedin.com/in/mehrab-mahdian/

License
-------

This project is licensed under the terms of the MIT license.
