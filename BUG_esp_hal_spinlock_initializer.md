# Bug Report: `clk_ctrl_os.c` uses invalid NuttX `spinlock_t` initializer

## Summary

When building openvela / NuttX for ESP32-S3 with the default `esp-hal-3rdparty` source selected by the build system, the build may fail in `clk_ctrl_os.c` because the NuttX lock object is initialized with integer `0`.

The problematic code is in Espressif's `esp-hal-3rdparty` component:

```c
#ifdef __NuttX__
#define LOCK_INITIALIZER_UNLOCKED       0

typedef spinlock_t lock_type_t;
static irqstate_t g_flags = UINT32_MAX;
#else
#define ENTER_CRITICAL_SECTION(lock)    portENTER_CRITICAL_SAFE(lock)
#define LEAVE_CRITICAL_SECTION(lock)    portEXIT_CRITICAL_SAFE(lock)
#define LOCK_INITIALIZER_UNLOCKED       portMUX_INITIALIZER_UNLOCKED

typedef portMUX_TYPE lock_type_t;
#endif

static lock_type_t periph_spinlock = LOCK_INITIALIZER_UNLOCKED;
```

For the NuttX path, `lock_type_t` is `spinlock_t`. In NuttX, `spinlock_t` is a structure type, so initializing it with bare `0` is invalid. The correct NuttX initializer is `SP_UNLOCKED`.

## Affected Path

Source path inside `esp-hal-3rdparty`:

```text
components/esp_hw_support/clk_ctrl_os.c
```

Path as seen during the NuttX / openvela build:

```text
arch/xtensa/src/chip/esp-hal-3rdparty/components/esp_hw_support/clk_ctrl_os.c
```

For ESP32-S3, `chip` usually resolves to the selected chip directory, for example:

```text
arch/xtensa/src/esp32s3/esp-hal-3rdparty/components/esp_hw_support/clk_ctrl_os.c
```

## Affected Configuration

The ESP32-S3 NuttX build pulls `esp-hal-3rdparty` according to:

```text
arch/xtensa/src/esp32s3/Make.defs
```

Relevant configuration:

```make
ESP_HAL_3RDPARTY_REPO = esp-hal-3rdparty
ESP_HAL_3RDPARTY_VERSION = 9fc713a95b1ff150dd0b0647e465d3c624056bb1
ESP_HAL_3RDPARTY_URL = https://github.com/espressif/esp-hal-3rdparty.git
```

The file is added to the ESP32-S3 build by:

```text
arch/xtensa/src/esp32s3/hal.mk
```

```make
CHIP_CSRCS += chip$(DELIM)$(ESP_HAL_3RDPARTY_REPO)$(DELIM)components$(DELIM)esp_hw_support$(DELIM)clk_ctrl_os.c
```

## Origin of the Problematic Code

The `LOCK_INITIALIZER_UNLOCKED 0` definition appears to have been introduced in `esp-hal-3rdparty` by this commit:

```text
ee8052d9c23c2bcb1a3768a5141144713a1a819e
nuttx: Add clk_ctrl_os.c port
```

The current ESP32-S3 NuttX configuration pins `esp-hal-3rdparty` to:

```text
9fc713a95b1ff150dd0b0647e465d3c624056bb1
```

That pinned version still contains the invalid NuttX initializer.

## Root Cause

In the NuttX branch of `clk_ctrl_os.c`, the code defines:

```c
typedef spinlock_t lock_type_t;
```

Then it creates a static lock object:

```c
static lock_type_t periph_spinlock = LOCK_INITIALIZER_UNLOCKED;
```

But `LOCK_INITIALIZER_UNLOCKED` is defined as:

```c
#define LOCK_INITIALIZER_UNLOCKED       0
```

This effectively becomes:

```c
static spinlock_t periph_spinlock = 0;
```

In NuttX, `spinlock_t` is not an integer. It is defined in:

```text
include/nuttx/spinlock_type.h
```

Depending on configuration, it may be a structure such as:

```c
typedef struct spinlock_s
{
  atomic_t lock;
} spinlock_t;
```

or a different structure layout, for example when ticket spinlocks or spinlock debug options are enabled.

NuttX provides the correct static initializer for this type:

```c
SP_UNLOCKED
```

`SP_UNLOCKED` expands to a valid initializer for the active NuttX spinlock configuration, such as `{0}`, `{0, 0}`, or another configuration-specific initializer.

## Expected Behavior

`clk_ctrl_os.c` should compile successfully when `__NuttX__` is defined.

The NuttX lock initializer should use the NuttX-provided `SP_UNLOCKED` macro.

## Actual Behavior

The build fails because `clk_ctrl_os.c` attempts to initialize a NuttX `spinlock_t` structure with integer `0`.

The failing source path is typically shown as:

```text
chip/esp-hal-3rdparty/components/esp_hw_support/clk_ctrl_os.c
```

The problematic macro is:

```c
#define LOCK_INITIALIZER_UNLOCKED       0
```

## Proposed Fix

Change the NuttX initializer from `0` to `SP_UNLOCKED`:

```diff
diff --git a/components/esp_hw_support/clk_ctrl_os.c b/components/esp_hw_support/clk_ctrl_os.c
--- a/components/esp_hw_support/clk_ctrl_os.c
+++ b/components/esp_hw_support/clk_ctrl_os.c
@@ -24,7 +24,7 @@
 #define LEAVE_CRITICAL_SECTION(lock) do { \
             spin_unlock_irqrestore((lock), g_flags); \
             g_flags = UINT32_MAX; \
         } while(0)
-#define LOCK_INITIALIZER_UNLOCKED       0
+#define LOCK_INITIALIZER_UNLOCKED       SP_UNLOCKED

 typedef spinlock_t lock_type_t;
 static irqstate_t g_flags = UINT32_MAX;
```

## Why This Fix Is Correct

`SP_UNLOCKED` is the official NuttX static initializer for `spinlock_t`.

It correctly adapts to different NuttX spinlock configurations, including:

- normal spinlock
- ticket spinlock
- spinlock debug mode
- zero-size spinlock fallback

This is also consistent with existing NuttX code patterns, for example:

```c
static spinlock_t g_lock = SP_UNLOCKED;
```

## Workaround

Manually patch `clk_ctrl_os.c`:

```c
#define LOCK_INITIALIZER_UNLOCKED       SP_UNLOCKED
```

However, this workaround may be overwritten if the build system re-fetches or checks out `esp-hal-3rdparty` again.

## Ownership / Bug Classification

This is primarily an upstream Espressif `esp-hal-3rdparty` bug because the invalid initializer is in Espressif's `esp-hal-3rdparty` repository and was introduced by the NuttX port of `clk_ctrl_os.c` there.

It can also be treated as an openvela / NuttX integration bug if openvela / NuttX pins and consumes an `esp-hal-3rdparty` revision that is known to fail with the default ESP32-S3 configuration. In that case, openvela / NuttX should either patch the vendored/generated HAL source during integration or update `ESP_HAL_3RDPARTY_VERSION` to a fixed upstream commit.

Recommended fix location:

1. Preferred: fix `components/esp_hw_support/clk_ctrl_os.c` in `espressif/esp-hal-3rdparty`.
2. Then: update the openvela / NuttX pinned `ESP_HAL_3RDPARTY_VERSION` to include the fix.
3. Temporary downstream workaround: carry a local patch in openvela / NuttX until upstream is fixed.
