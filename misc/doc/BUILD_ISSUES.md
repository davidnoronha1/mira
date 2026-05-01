# Build Issues & Fixes

Common build failures and how to resolve them.

---

## Switching between Docker and native builds

Building inside Docker and then building natively (or vice versa) leaves behind
CMake cache entries and compiled artifacts with mismatched paths and toolchain
settings, causing cryptic build failures.

**Fix:** Clean and rebuild from scratch:
```
python mira.py clean
python mira.py build
```

---

## Permission errors after a Docker build

Files written inside a Docker container are owned by `root`. When you then build
natively, colcon cannot overwrite those files and the build fails with permission
denied errors.

**Fix:** Restore workspace ownership to your user, then clean and rebuild:
```
python mira.py docker-fix-perms
python mira.py clean
python mira.py build
```

---

## CMakeCache / build-install-log mismatch

Errors mentioning `CMakeCacheList`, mismatched paths, or stale entries in `build/`,
`install/`, or `log/` mean the CMake cache is out of sync with the current source.
This often happens after moving the workspace directory or switching branches that
change CMake configuration.

**Fix:**
```
python mira.py clean
python mira.py build
```

---

## Missing or broken venv / Python import errors

If colcon prints Python import errors or the venv looks incomplete, the virtual
environment may be missing or out of date.

**Fix:**
```
python mira.py install-deps
```

---

## ROS dependency not found (rosdep)

If colcon cannot locate a ROS package that should be installed system-wide, run
rosdep to pull in missing system packages:

```
python mira.py install-deps
```

---

## Missing packages or submodule errors (camera_driver, zed_msgs, etc.)

If colcon reports that a package such as `camera_driver`, `zed_msgs`, or another
dependency cannot be found, the Git submodules have likely not been initialised.

**Fix:**
```
git submodule update --init --recursive
```
