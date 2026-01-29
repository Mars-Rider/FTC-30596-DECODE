//Websocket
/*var connection = new WebSocket("ws://" + "192.168.4.1" + ":81/", ["arduino"]);

connection.onopen = function () {
  //connection.send("Connect " + new Date());
  console.log("Connected");
};

connection.onerror = function (error) {
  alert("WebSocket Error ", error);
};

connection.onmessage = function (event) {
  try {
    const data = JSON.parse(event.data); // parse JSON from ESP8266
    jsonOutputEl.textContent = JSON.stringify(data, null, 2);

    // Example: update DOM based on LED channels
    // data is an array of channels
    data.forEach((channel, index) => {
      console.log(
        `Channel ${index}: h=${channel.h}, brightness=${channel.brightness}`
      );
    });

    strips = data;
  } catch (e) {
    console.error("Error parsing JSON:", e);
  }
};
*/

const STRIP_COUNT = 5;

var strips = [
  {
    numLEDs: 30,
    h: 0,
    s: 255,
    brightness: 255,
    rainbow: true,
    colorPeriod: 10000,
    hueMin: 0,
    hueMax: 270,
    pattern: 3,
    trailLength: 10,
    sourceCenter: 1,
    gradient: true,
    moving: true,
    period: 2000,
    direction: true,
  },
  {
    numLEDs: 30,
    h: 300,
    s: 255,
    brightness: 255,
    rainbow: false,
    colorPeriod: 10000,
    hueMin: 0,
    hueMax: 255,
    pattern: 4,
    trailLength: 20,
    sourceCenter: 10,
    gradient: true,
    moving: true,
    period: 1000,
    direction: true,
  },
  {
    numLEDs: 30,
    h: 120,
    s: 255,
    brightness: 255,
    rainbow: false,
    colorPeriod: 10000,
    hueMin: 0,
    hueMax: 255,
    pattern: 1,
    trailLength: 20,
    sourceCenter: 10,
    gradient: true,
    moving: true,
    period: 1200,
    direction: true,
  },
  {
    numLEDs: 10,
    h: 100,
    s: 255,
    brightness: 255,
    rainbow: false,
    colorPeriod: 10000,
    hueMin: 0,
    hueMax: 255,
    pattern: 7,
    trailLength: 0,
    sourceCenter: 10,
    gradient: true,
    moving: true,
    period: 1200,
    direction: true,
  },
];

let time = 0;
let offBrightness = 0.2;

function hsl(h, s, v) {
  s /= 100;
  v /= 100;

  const l = v * (1 - s / 2);
  const sl = l === 0 || l === 1 ? 0 : (v - l) / Math.min(l, 1 - l);

  return `hsl(${h}, ${(sl * 100).toFixed(1)}%, ${(l * 100).toFixed(1)}%)`;
}

function identifyPattern(i) {
  switch (i) {
    case 1:
      return "SOLID";
    case 2:
      return "BLINK";
    case 3:
      return "SCANNER";
    case 4:
      return "BREATHE";
    case 5:
      return "GRADIENT";
    case 6:
      return "MOVING GRADIENT";
    case 7:
      return "PING-PONG";
    case 8:
      return "FLICKER";
    case 9:
      return "HEARTBEAT";
    case 10:
      return "RADAR";
    case 11:
      return "SLIDER";
    case 12:
      return "BOUNCING SLIDER";
    case 13:
      return "WAVE";
  }
}

function render() {
  const grid = document.getElementById("grid");
  grid.innerHTML = "";

  for (let i = 0; i < STRIP_COUNT; i++) {
    const s = strips[i];
    const card = document.createElement("div");
    card.className = "card" + (!s ? " disconnected" : "");

    card.innerHTML = `
          <div class="header">
            <strong>Strip ${i + 1}</strong>
            <div class="badges">
              <span class="badge">${
                s ? identifyPattern(s.pattern) : "DISCONNECTED"
              }</span> 
              ${
                s
                  ? `<span class="badge">${
                      s.rainbow
                        ? "RAINBOW - " + Math.round((s.brightness / 255) * 100) + "%"
                        : "HSV: " + s.h + ", " + Math.round((s.s / 255)) * 100 + "%, 100%"
                    }</span>
              <span class="badge">${
                "Brightness: " + Math.round((s.brightness / 255) * 100) + "%"
              }</span >`
                  : ""
              }
            </div>
          </div>
          <div class="strip"></div>
        `;
    grid.appendChild(card);
  }
}

function animate() {
  document.querySelectorAll(".card").forEach((card, i) => {
    const strip = card.querySelector(".strip");
    const s = strips[i];
    if (!s) {
      strip.style.background = "#333";
      return;
    }

    function baseColor(S = 1, v = 1) {
      if (s.rainbow) {
        return hsl(
          ((time * 1000 * 1000) / s.colorPeriod) % 360,
          S * 100,
          v * 100
        );
      } else {
        return hsl(s.h, (s.s / 255) * S * 100, (s.brightness / 255) * v * 100);
      }
    }

    function color(H, S = 1, v = 1) {
        return hsl(H, (s.s / 255) * S * 100, (s.brightness / 255) * v * 100);
    }

    switch (identifyPattern(s.pattern)) {
      case "SOLID":
        strip.style.background = baseColor();
        break;

      case "BLINK":
        const l = Math.floor(((time * 1000) / s.period) % 2);
        strip.style.background = baseColor(
          l,
          (1 - offBrightness) * l + offBrightness
        );
        break;

      case "BREATHE":
        const t = (Math.sin(time * ((2000 * Math.PI) / s.period)) + 1) / 2;
        strip.style.background = baseColor(
          t,
          (1 - offBrightness) * t + offBrightness
        );
        break;

      case "SCANNER": {
        const phase = ((time * 1000) % s.period) / s.period;
        var pos =
          phase *
          (1*100) +
            (1 -
            (0 -
              (100 / s.numLEDs) * s.sourceCenter -
              (100 / s.numLEDs) * s.trailLength));

        if (s.direction == 1) {
          strip.style.background = `
              linear-gradient(90deg,
              ${baseColor(0, offBrightness)} ${
            pos -
            (100 / s.numLEDs) * s.sourceCenter -
            (100 / s.numLEDs) * s.trailLength - 100
          }%,
              ${baseColor()} ${pos - (100 / s.numLEDs) * s.sourceCenter - 100}%,
                ${baseColor()} ${pos - 100}%,${baseColor(0, offBrightness)} ${
            pos + 100 / s.numLEDs + 1 - 100
          }%,
                ${baseColor(0, offBrightness)} ${
            pos -
            (100 / s.numLEDs) * s.sourceCenter -
            (100 / s.numLEDs) * s.trailLength
          }%,
                ${baseColor()} ${pos - (100 / s.numLEDs) * s.sourceCenter}%,
                ${baseColor()} ${pos}%,
                ${baseColor(0, offBrightness)} ${pos + 100 / s.numLEDs + 1}%
          )
            `;
        } else {
            pos =
              -phase * (1 * 100) +
              (1 +
                (0 +
                  (100 / s.numLEDs) * s.sourceCenter +
                  (100 / s.numLEDs) * s.trailLength));
            strip.style.background = `
              linear-gradient(90deg,
                ${baseColor(0, offBrightness)} ${pos - 100 / s.numLEDs - 1}%,
                ${baseColor()} ${pos}%,
                ${baseColor()} ${pos + (100 / s.numLEDs) * s.sourceCenter}%,
                ${baseColor(0, offBrightness)} ${
              pos +
              (100 / s.numLEDs) * s.sourceCenter +
              (100 / s.numLEDs) * s.trailLength
            }%,
                ${baseColor(0, offBrightness)} ${pos - 100 / s.numLEDs - 1 + 100}%,
                ${baseColor()} ${pos + 100}%,
                ${baseColor()} ${pos + (100 / s.numLEDs) * s.sourceCenter + 100}%,
                ${baseColor(0, offBrightness)} ${
              pos +
              (100 / s.numLEDs) * s.sourceCenter +
              (100 / s.numLEDs) * s.trailLength + 100
            }%)`;
        }
        break;
      }
      case "PING-PONG": {
        const phase = ((time * 1000) % s.period) / s.period;
        const pos = Math.abs(phase * 2 - 1) * 100;
        if (phase * 2 - 1 <= 0) {
          strip.style.background = `
              linear-gradient(90deg,
                ${baseColor(0, offBrightness)} ${
            pos - 100 / s.numLEDs - (100 / s.numLEDs) * s.trailLength
          }%,
                ${baseColor()} ${pos - 100 / s.numLEDs}%,
                ${baseColor()} ${pos}%,${baseColor(0, offBrightness)} ${
            pos + 100 / s.numLEDs + 1
          }%)
            `;
        } else {
          strip.style.background = `
              linear-gradient(90deg,
                ${baseColor(0, offBrightness)} ${pos - 100 / s.numLEDs - 1}%,
                ${baseColor()} ${pos}%,${baseColor()} ${
            pos + 100 / s.numLEDs
          }%,${baseColor(0, offBrightness)} ${
            pos + (100 / s.numLEDs) * s.trailLength
          }%)
            `;
        }
        break;
      }
      case "SLIDER": {
        const phase = ((time * 1000) % s.period) / s.period;
        const pos = phase * 2 * 100;
        strip.style.background = `
              linear-gradient(90deg,
                ${baseColor(0, offBrightness)} ${pos - s.trail}%,
                ${baseColor()} ${pos}%)
            `;
        break;
      }

      case "BOUNCING SLIDER": {
        const phase = ((time * 1000) % s.period) / s.period;
        const pos = Math.abs(phase * 2 - 1 * 100);
        strip.style.background = `
              linear-gradient(90deg,
                ${baseColor(0, offBrightness)} ${pos - s.trail}%,
                ${baseColor()} ${pos}%)
            `;
        break;
      }

      case "HEARTBEAT":
        brightness *= heartbeatBrightness(time * 1000, s.period);
        strip.style.background = baseColor();
        strip.style.filter = `brightness(${brightness})`;
        break;

      case "GRADIENT":
        if(s.rainbow && !s.moving){

            strip.style.background = `
              linear-gradient(in hsl longer hue 90deg,
                ${color(0, 1, 1)} 0 100%)
            `;
        } else if (s.rainbow && s.moving) {

          strip.style.background = `
              linear-gradient(in hsl longer hue 90deg,
                ${baseColor(1, 1)} 0 100%)
            `;
        } else if (!s.rainbow && s.moving) {
            const phase = ((time * 1000) % s.period) / s.period;
            var pos = phase * 2 * 100;

            console.log(pos);

          strip.style.background = `
              linear-gradient(90deg,
                ${color(s.hueMin, 1, 1)} ${pos+100 - (200 / 2) * 3}%,
                ${color(s.hueMax, 1, 1)} ${pos+100 - (200 / 2) * 2}%,
                ${color(s.hueMin, 1, 1)} ${pos+100 - (200 / 2) * 1}%, 
                ${color(s.hueMax, 1, 1)} ${pos+100}%)
            `;
        } else {
          strip.style.background = `
              linear-gradient(90deg,
                ${color(s.hueMin, 1, 1)} ${0}%,
                ${color(s.hueMax, 1, 1)} ${75}%, 
                ${color(s.hueMin, 1, 1)} ${150}%)
            `;
        }
        break;
    }
  });

  time += 0.016;
  requestAnimationFrame(animate);
}

function heartbeatBrightness(t, period) {
  const p = (t % period) / period;
  if (p < 0.08) return 1;
  if (p < 0.16) return 0.3;
  if (p < 0.25) return 0.9;
  return 0.15;
}

render();
animate();
