# Camera Calibration Files

Place camera calibration JSON files here when the iPhone 13 rear-camera phone workflow has been calibrated.

The default phone calibration workflow writes:

```text
software/companion/vision/calibration/iphone13_rear_checkerboard.json
```

Expected JSON format:

```json
{
  "camera_matrix": [
    [1000.0, 0.0, 640.0],
    [0.0, 1000.0, 360.0],
    [0.0, 0.0, 1.0]
  ],
  "dist_coeffs": [0.0, 0.0, 0.0, 0.0, 0.0]
}
```

Until a calibration file is supplied, phone tracking can still detect the A4 tag board, but it does not mark height as a valid calibrated pose.
