# RoboFest perception

`vision_pipeline_node` produces exactly two image labels:

- `mine`: a configured red or yellow mine marker with a compact shape;
- `object`: any other sufficiently large foreground region, without trying to
  identify its class.

The complete result is published on `/perception/detections` as
`robofest_interfaces/DetectionMap`. Every result includes a label, confidence,
pixel bounding box, and a ground position when ToF range and attitude data are
available. `/perception/detection_labels` provides a compact human-readable
summary such as `mine:1,object:2`.

Only `mine` results are forwarded to the existing
`/perception/mine_detections` topic. They are sent in `base_link` coordinates,
matching the current mine-mapping subscriber. Generic objects therefore remain
visible to perception consumers without being inserted into the mine map or
changing the mine-only path planner.

Tune the HSV bands and contrast/area thresholds in `config/vision.yaml` for the
actual camera and field surface. The detector has no network or model-file
dependency and can be tested offline with:

```bash
PYTHONPATH=ros2_ws/src/robofest_perception \
  python3 -m unittest discover -s ros2_ws/src/robofest_perception/test -v
```
