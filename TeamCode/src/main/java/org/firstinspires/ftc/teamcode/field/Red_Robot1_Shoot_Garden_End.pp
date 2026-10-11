{
  "startPoint": {
    "x": 56,
    "y": 8,
    "locked": false,
    "headingDeg": 90,
    "name": "RED_START"
  },
  "lines": [
    {
      "id": "line-mv2tet1u-k6mzpc",
      "color": "#ffc516",
      "name": "RED_START_TO_GARDEN",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 3500,
      "waitBeforeName": "",
      "waitAfterName": "",
      "kind": "atomic",
      "endPoint": {
        "x": 20,
        "y": 8
      },
      "controlPoints": [
        {
          "x": 56,
          "y": 20
        }
      ],
      "heading": {
        "type": "linear",
        "startDeg": 90,
        "endDeg": 180,
        "reverse": false,
        "piecewiseHeading": {
          "segments": [
            {
              "startProgress": 0,
              "endProgress": 1,
              "interpolationType": "linear",
              "reversed": false,
              "parameters": {
                "startDeg": 0,
                "endDeg": 0
              }
            }
          ]
        }
      }
    },
    {
      "kind": "atomic",
      "id": "line-mv460ktr-p5y1fb",
      "endPoint": {
        "x": 12,
        "y": 97
      },
      "controlPoints": [
        {
          "x": 24,
          "y": 24
        }
      ],
      "heading": {
        "type": "tangential",
        "reverse": false,
        "startDeg": 0,
        "endDeg": 90,
        "piecewiseHeading": {
          "segments": [
            {
              "startProgress": 0,
              "endProgress": 1,
              "interpolationType": "linear",
              "reversed": false,
              "parameters": {
                "startDeg": 0,
                "endDeg": 0
              }
            }
          ]
        }
      },
      "color": "#BC6666",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "name": "RED_GARDEN_TO_PARK"
    }
  ],
  "shapes": [
    {
      "id": "triangle-1",
      "name": "Red Goal",
      "vertices": [
        {
          "x": 141.5,
          "y": 70
        },
        {
          "x": 141.5,
          "y": 141.5
        },
        {
          "x": 118.3,
          "y": 141.5
        },
        {
          "x": 135.5,
          "y": 118
        },
        {
          "x": 136.3,
          "y": 70.2
        }
      ],
      "color": "#dc2626",
      "fillColor": "#ff6b6b"
    },
    {
      "id": "triangle-2",
      "name": "Blue Goal",
      "vertices": [
        {
          "x": 6.2,
          "y": 116.9
        },
        {
          "x": 25,
          "y": 141.5
        },
        {
          "x": 0,
          "y": 141.5
        },
        {
          "x": 0,
          "y": 70
        },
        {
          "x": 6,
          "y": 70
        }
      ],
      "color": "#2563eb",
      "fillColor": "#60a5fa"
    }
  ],
  "sequence": [
    {
      "kind": "path",
      "lineId": "line-mv2tet1u-k6mzpc"
    },
    {
      "kind": "path",
      "lineId": "line-mv460ktr-p5y1fb"
    }
  ],
  "fieldPoints": [],
  "activePaths": [],
  "settings": {
    "xVelocity": 75,
    "yVelocity": 65,
    "aVelocity": 3.141592653589793,
    "kFriction": 0.1,
    "rWidth": 16,
    "rHeight": 16,
    "safetyMargin": 1,
    "maxVelocity": 40,
    "maxAcceleration": 30,
    "maxDeceleration": 30,
    "fieldMap": "biobuzz.webp",
    "robotImage": "/robot.png",
    "showGhostPaths": false,
    "showOnionLayers": false,
    "onionLayerSpacing": 3,
    "onionColor": "#dc2626",
    "onionNextPointOnly": false,
    "showHeadingArrow": false,
    "showCurrentTValue": false,
    "leftPanelWidth": 0,
    "rightPanelWidth": 695,
    "headingArrowLength": 50,
    "headingArrowColor": "#ffffff",
    "headingArrowThickness": 2,
    "pathOpacity": 1,
    "leftPanelMinWidth": 0,
    "rightPanelMinWidth": 0,
    "penToolMaxPaths": 8,
    "curveThroughMaxPoints": 4,
    "experimentalFeatures": {
      "optimize": false,
      "curveThrough": false
    }
  },
  "version": "1.5.0",
  "timestamp": "2026-10-11T18:43:58.889Z"
}