# Damiao CAN Python bindings

Python bindings for the DaMiao CAN motor-control library.

> [!WARNING]
>
> ⚠️ **WARNING: UNSTABLE API** ⚠️
> Python bindings are currently a direct low level **temporary port**, and will change **DRASTICALLY**.
> The interface is may break between versions.Use at your own risk! Discussions on the interface are welcomed.

## Motor initialization

`motor_types` is optional. Omitting it enables register-based auto initialization:

```python
import damiao_can as dc

device = dc.DamiaoCAN("can0", True)
device.init_motors(
    [0x01],
    [0x11],
    control_modes=[dc.ControlMode.MIT],
)
```

Auto initialization reads `PMAX`, `VMAX`, and `TMAX` from each motor and uses them as that motor's protocol limits. If the limits cannot be resolved safely, `MotorLimitResolutionError` is raised. Each register read uses a 100 ms maximum response timeout and returns as soon as the reply arrives. Pass `motor_types=[dc.MotorType.DM4310]` to override auto detection, or use `None` entries for per-motor auto detection in a mixed list.

