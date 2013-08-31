2D Hovering Rocket with TVC (Thrust Vector Control) engine.
---
![Visuals](http://i.imgur.com/pcjQyup.png)

The system is controlled by 3 PD controllers.

The first controls throttle based on altitude.
The second controls TVC angle based on orientation.
The third modifies TVC angle based on lateral drift.


The rocket is modelled as a simple 5.5x33m box massing 10 tons, with a maximum thrust of 500 kN.
A blue line shows the force in the normal direction.
Thrust is shown as a red line, which is in the direction dictated by the TVC controllers.
![Components](http://i.imgur.com/l0gWlgG.png)

Pressing 'i' for info returns PID values:

```
    PID Values:
       lateral drift : PID(1.5,0,5.25)
            altitude : PID(0.3,0,0.67)
            attitude : PID(0.4,0,0.2)
```
