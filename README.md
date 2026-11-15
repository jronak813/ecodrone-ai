# EcoDrone AI – Drone Plastic Zone Scanner

**Live website:** https://jronak813.github.io/ecodrone-ai/

EcoDrone AI scans land from a drone photo or a live camera and shows where plastic waste is.

- Finds **macroplastic** (big pieces) and **microplastic** (tiny specks) in the picture
- Colours the land in **Red / Orange / Green** zones (High / Medium / Low plastic)
- Shows every scanned place on a **map** of Udaipur and Rajasthan
- Uses an **RGB colour sensor** and a **light sensor** (Arduino) to make the result more reliable
- Sends a **WhatsApp alert** with the zone image, numbers and location
- Works offline and can be installed on a phone like an app

## Files
| File | What it is |
|---|---|
| `index.html` | The whole app |
| `ecodrone_sensors.ino` | Arduino code for the TCS3200 colour sensor + LDR light sensor |
| `sw.js`, `manifest.json`, icons | Offline support and “Install app” |

## Zone rules
| Zone | Macroplastic | Microplastic |
|---|---|---|
| 🔴 Red (High) | 5% of photo or more, or 15+ pieces | 0.4% or more |
| 🟠 Orange (Medium) | 1% or more, or 5+ pieces | 0.1% or more |
| 🟢 Green (Low) | less | less |

A drone photo cannot see plastic smaller than a few millimetres, so the microplastic value is a screening estimate. Always confirm on site.
