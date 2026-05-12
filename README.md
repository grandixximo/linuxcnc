
[![Badge GPL2]][License]
[![Badge LGPL]][License]

<div align = center>

<br>

# LinuxCNC with S-Curve Motion Planning

*Controlling CNC Machines - Now with Smoother Motion*

<br>

## ⚠️ **IMPORTANT: THIS IS A MODIFIED VERSION** ⚠️

<br>

</div>

---

## 🔥 **NEW FEATURE: S-Curve Trajectory Planner**

This fork adds **optional S-curve (jerk-limited) motion planning** to LinuxCNC, providing smoother acceleration profiles that can reduce vibration and improve surface finish.

### **⚡ Quick Start - How to Activate S-Curve**

**S-curve planning is DISABLED by default.** To enable it, add these lines to your INI file:

```ini
[TRAJ]
PLANNER_TYPE = 1              # 0 = Traditional (default), 1 = S-curve
MAX_LINEAR_JERK = 1000.0      # Adjust based on your machine
DEFAULT_LINEAR_JERK = 500.0   # Default jerk for moves

[JOINT_0]
MAX_JERK = 1000.0             # Set for each joint

[AXIS_X]
MAX_JERK = 1000.0             # Set for each axis
```

**Without these settings, this version runs EXACTLY like standard LinuxCNC!**

### **📚 Documentation**

- **Quick Reference:** See [SCURVE_QUICKREF.txt](SCURVE_QUICKREF.txt) for complete guide
- **Example Configuration:** See [configs/sim/axis/axis_mm_scurve.ini](configs/sim/axis/axis_mm_scurve.ini)
- **HAL Examples:** See [configs/scurve_runtime_hal_examples.hal](configs/scurve_runtime_hal_examples.hal)

### **✨ Key Features**

- ✅ **Optional** - Disabled by default, backward compatible
- ✅ **Runtime Switching** - Toggle between trapezoidal and S-curve via HAL pins
- ✅ **Configurable** - Set jerk limits per trajectory/axis/joint
- ✅ **Smoother Motion** - Reduces vibration and mechanical stress
- ✅ **Better Surface Finish** - Especially beneficial for finishing passes

### **🎯 Typical Jerk Values**

- **Light/rigid machines:** 1,000 - 10,000 units/s³
- **Medium machines:** 100 - 1,000 units/s³
- **Heavy/flexible machines:** 100 - 500 units/s³

*Units depend on your LINEAR_UNITS setting (mm or inch)*

### **⚙️ Runtime Control**

Switch planners on-the-fly using HAL pins:
```
setp ini.traj_planner_type 1      # Enable S-curve
setp ini.traj_max_jerk 2000.0     # Adjust jerk dynamically
```

---

<div align = center>

<br>

[![Badge Translation]][Translation]

<br>

---

[<kbd> <br> Ｗｅｂｓｉｔｅ <br> </kbd>][Website]
[<kbd> <br> Ｉｎｓｔａｌｌ <br> </kbd>][Ｉｎｓｔａｌｌ]
[<kbd> <br> Ｂｕｉｌｄ <br> </kbd>][Ｂｕｉｌｄ]
[<kbd> <br> Ｄｏｃｕｍｅｎｔａｔｉｏｎ <br> </kbd>][Ｄｏｃｕｍｅｎｔａｔｉｏｎ]

---

<br>

## About LinuxCNC

It can drive milling machines, lathes, 3D printers, laser <br>
cutters, plasma cutters, robot arms, hexapods, and more.

LinuxCNC was initiated 25 years ago and evolved into a very <br>
international project with contributions from all over the globe.

With release 2.9 of LinuxCNC we also transitioned the <br>
documentation to the use of the public crowd translation <br>
services [Weblate] and invite all our users to contribute.

The translations we expect to help attract practitioners <br>
to the project and also helps educating enthusiasts of <br>
all age groups on automated machining.

<br>

## DISCLAIMER

<br>

```

Ｔｈｅ ａｕｔｈｏｒｓ ｏｆ ｔｈｉｓ ｓｏｆｔｗａｒｅ ａｃｃｅｐｔ
ａｂｓｏｌｕｔｅｌｙ ｎｏ ｌｉａｂｉｌｉｔｙ ｆｏｒ ａｎｙ
ｈａｒｍ　ｏｒ ｌｏｓｓ ｒｅｓｕｌｔｉｎｇ ｆｒｏｍ ｉｔｓ ｕｓｅ．

Ｉｔ ｉｓ ＥＸＴＲＥＭＥＬＹ ｕｎｗｉｓｅ ｔｏ　ｒｅｌｙ
ｏｎ ｓｏｆｔｗａｒｅ ａｌｏｎｅ ｆｏｒ ｓａｆｅｔｙ．

Any machinery capable of harming persons must have
provisions for completely removing power from all
motors, etc., before persons enter any danger area.

All machinery must be designed to comply with local
and national safety codes, and the authors of this
software cannot and do not, take any responsibility
for such compliance.

```

<br>

## 🧪 S-Curve Feature Status

This S-curve implementation is:
- ✅ Functionally complete
- ⚠️ **Experimental** - Community testing needed
- 🔒 Safe by default (disabled unless explicitly configured)

**Please test and report your experience!**

Report issues or feedback:
- GitHub Issues: [Create an issue](../../issues)
- LinuxCNC Forum: Post your results and jerk values that worked
- Share your machine specs and optimal settings to help others

<br>

</div>

<!----------------------------------------------------------------------------->

[Badge Translation]: https://hosted.weblate.org/widgets/linuxcnc/-/svg-badge.svg
[Badge GPL2]: https://img.shields.io/badge/Most-LGPL_3-blue.svg?style=for-the-badge 'The license this software is under'
[Badge LGPL]: https://img.shields.io/badge/Some-GPL_2-blue.svg?style=for-the-badge 'Some parts are under this license'

[Translation]: https://hosted.weblate.org/engage/linuxcnc/
[Weblate]: https://hosted.weblate.org/projects/linuxcnc/
[Website]: https://linuxcnc.org/

[Ｄｏｃｕｍｅｎｔａｔｉｏｎ]: http://linuxcnc.org/docs/devel/html/
[Ｉｎｓｔａｌｌ]: http://linuxcnc.org/docs/devel/html/getting-started/getting-linuxcnc.html
[Ｂｕｉｌｄ]: http://linuxcnc.org/docs/devel/html/code/building-linuxcnc.html
[License]: COPYING
