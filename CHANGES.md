# Changes from upstream (Wired-Square/esphome-twc-director)

## Fix: UNCONF_PERIPHERAL deadlock in `send_heartbeat()` — `twc_core.c`

**File:** `components/twc_director/twc/twc_core.c`  
**Function:** `send_heartbeat()`

### Problem

The TWC Gen 2 boots in `TWC_MODE_UNCONF_PERIPHERAL` state, signalled by status
byte `0x80` in every E2 response frame. The original code sent all-zeros in the
heartbeat (E0) for any device in `UNCONF_PERIPHERAL` mode:

```c
// ORIGINAL — broken
if (mode == TWC_MODE_UNCONF_PERIPHERAL) {
  charge_state = 0u;
  available_centiamps = 0u;
  delivered_centiamps = 0u;
} else {
  // pending command logic — never reached for UNCONF_PERIPHERAL
}
```

However the TWC requires a **non-zero current offer** (`state=0x05`, `avail>0`)
before it will transition out of the unconfigured state. This created a
permanent deadlock:

```
director sends zeros  →  TWC stays UNCONF_PERIPHERAL  →  director sends zeros  →  …
```

Observed symptom: `state=0x00 avail=0 deliv=0` in every heartbeat log line
indefinitely; TWC never comes online in Home Assistant.

### Fix

For `UNCONF_PERIPHERAL` devices, allow `pending_initial_current_cmd` to
proceed so the director sends a `0x05` frame with the configured current.
All other pending commands (session `0x09`, increase `0x06`, decrease `0x07`)
remain blocked until the TWC is fully configured.

```c
// PATCHED
if (mode == TWC_MODE_UNCONF_PERIPHERAL) {
  if (dev->pending_initial_current_cmd) {
    charge_state = 0x05u;
    available_centiamps = (uint16_t)(current_a * 100.0f + 0.5f);
    delivered_centiamps = 0u;
  } else {
    charge_state = 0u;
    available_centiamps = 0u;
    delivered_centiamps = 0u;
  }
} else {
  // full pending command logic as before
}
```

Once the TWC receives the current offer it transitions to `TWC_MODE_PERIPHERAL`,
the info-probe sequence begins, and the device comes online in Home Assistant.
