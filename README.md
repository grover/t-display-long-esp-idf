<h1 align = "center">🌟LilyGO T-Display-S3-Long - pure ESP-IDF template 🌟</h1> 

This is a fork of the LVGL sample in [LilyGOs T-Display-S3-Long repository][2] using only ESP-IDF instead of the Arduino framework on top of ESP-IDF. This works exclusively in Platform IO and offers a much more native ESP32 experience compared to the Arduino IDE.


## 1️⃣Product

| Product(PinMap)        | SOC        | Flash | PSRAM    | Resolution |
| ---------------------- | ---------- | ----- | -------- | ---------- |
| [T-Display-S3-Long][1] | ESP32-S3R8 | 16MB  | 8MB(OPI) | 180x640    |

| Current consumption    | Working current             | sleep current | sleep mode  |
| ---------------------- | --------------------------- | ------------- | ----------- |
| [T-Display-S3-Long][1] | (240MHz) WiFi On 90~350+ mA | About 1.1mA   | gpio wakeup |

[1]:https://www.lilygo.cc/products/t-display-s3-long
[2]:https://github.com/Xinyuan-LilyGO/T-Display-S3-Long