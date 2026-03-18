# pedal_generator_gearbox

Pedal-generator gearbox exploration for the **ProjectNeoDrive pedal-by-wire** concept.

## Goal

This project is used to find a **decent gearbox configuration** for a pedal generator that:
- is cheap
- stays reasonably backdrivable,
- minimizes wear,
- keeps efficiency acceptable,
- and works with a **BLDC motor around Kv=100** (the motor class we already have in stock).

The current script searches valid gear train combinations and ranks them with a weighted score (backdrivability, wear, efficiency, and stage count/cost), then plots expected regen efficiency over pedal RPM and battery SOC.

## Main script

- `gear.py`

Run:

```bash
python3 gear.py
```

Outputs include:
- ranked gearbox candidates,
- `best_match_efficiency_map.png`,
- `best_match_efficiency_map_kv_compare.png`.

---

![plot generated for gear efficiency](best_match_efficiency_map.png)
![taking KV into account](best_match_efficiency_map_kv_compare.png)

## Sourcing links (initial shortlist)

> Note: AliExpress listings can change over time (title, specs, stock, URL behavior).

### BLDC motor candidates

- https://de.aliexpress.com/item/1005009622680232.html
- https://de.aliexpress.com/item/1005010776963168.html
- https://de.aliexpress.com/item/1005010041210790.html
- https://de.aliexpress.com/item/1005011728500156.html
- https://de.aliexpress.com/item/1005008043040484.html

### Gear candidates (module 1 steel spur gears)

- 70T: https://de.aliexpress.com/item/1005004090227296.html
- 60T: https://de.aliexpress.com/item/1005004090213302.html
- 38T: https://de.aliexpress.com/item/1005004090075755.html
- 25T: https://de.aliexpress.com/item/1005004089851114.html
- 21T: https://de.aliexpress.com/item/1005004089721227.html
- 18T: https://de.aliexpress.com/item/1005004089640535.html
- 12T: https://de.aliexpress.com/item/1005004088412911.html

### Full pedal + gearbox / pedal assembly candidates

- Foot pedal + reduction gearbox assembly: https://de.aliexpress.com/item/1005004857033022.html
- BMX crank + pedal set (mechanical pedal source): https://de.aliexpress.com/item/1005007805733704.html
- Bicycle pedal set: https://de.aliexpress.com/item/1005001871077322.html


---
