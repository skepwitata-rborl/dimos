

<details><summary>Python</summary>

```python fold session=mem output=none
import pickle
from dimos.mapping.pointclouds.occupancy import general_occupancy, simple_occupancy, height_cost_occupancy
from dimos.mapping.occupancy.inflation import simple_inflate
from dimos.memory2.store.sqlite import SqliteStore
from dimos.memory2.vis.color import color
from dimos.memory2.transform import downsample, throttle, speed, smooth
from dimos.memory2.vis.drawing.drawing import Drawing2D as Drawing
from dimos.utils.data import get_data
from dimos.memory2.vis.type import Point
```

</details>


we init our recording, investigate available streams

```python session=mem
store = SqliteStore(path=get_data("go2_bigoffice.db"))

for name, stream in store.streams.items():
   print(stream.summary())
```

<!--Result:-->
```
Stream("color_image"): 4164 items, 2025-12-26 11:09:07 — 2025-12-26 11:13:59 (292.5s)
Stream("color_image_embedded"): 267 items, 2025-12-26 11:09:11 — 2025-12-26 11:13:59 (288.4s)
Stream("lidar"): 2263 items, 2025-12-26 11:09:06 — 2025-12-26 11:14:00 (293.9s)
```

Any stream is drawable

```python session=mem output=none
global_map = pickle.loads(get_data("unitree_go2_bigoffice_map.pickle").read_bytes())

drawing = Drawing()

# this is not neccessary but we use a global map as a nice base for a drawing
drawing.add(global_map)
drawing.add(store.streams.color_image)
drawing.to_svg("assets/color_image.svg")
```


our drawing system applies turbo color scheme to timestamps by default

![output](assets/color_image.svg)

we can create new streams by querying existing streams, and we can save, further transform or draw those

```python session=mem output=none

drawing = Drawing()
drawing.add(global_map)

drawing.add(
  store.streams.color_image \
  # calculate speed in m/s by checking distance between poses and timestamps of observations
  .transform(speed()) \
  # rolling window average
  .transform(smooth(50)))

drawing.to_svg("assets/speed.svg")
```


![output](assets/speed.svg)

we can do all kinds of things with this, for example map out room lighting

```python session=mem output=none
drawing = Drawing()
drawing.add(global_map)

drawing.add(
  store.streams.color_image \
  # here we will take 4fps because brigtness calculation loads the actual image
  # observation.data triggers another db query to fetch the data
  # otherwise observations only hold positions and timestamps
  .transform(throttle(0.25)) \
  # we calculate brightness
  .map(lambda obs: obs.derive(data=obs.data.brightness)))

drawing.to_svg("assets/brightness.svg")
```


![output](assets/brightness.svg)

So knowing above, we can create embeddings for the full stream,

```python session=mem skip
from dimos.models.embedding.clip import CLIPModel
from dimos.msgs.sensor_msgs.Image import Image
from dimos.memory2.transform import QualityWindow
from dimos.memory2.embed import EmbedImages

embedded = store.stream("color_image_embedded", Image)
clip = CLIPModel()

# Downsample to 2Hz, filter dark images, then embed
pipeline = (
    store.streams.color_image.filter(lambda obs: obs.data.brightness > 0.1)
    .transform(QualityWindow(lambda img: img.sharpness, window=0.5))
    .transform(EmbedImages(clip))
    .save(embedded)
)

print(pipeline)

```

this pipeline is ready to execute by lazy, we can execute it by iterating, or calling .drain()

```python skip
for obs in pipeline:
    print(f"  [{count}] ts={obs.ts:.2f} pose={obs.pose}")
```

let's query it!


```python session=mem output=none
from dimos.models.embedding.clip import CLIPModel

drawing = Drawing()
drawing.add(global_map)

clip = CLIPModel()
drawing.add(store.streams.color_image_embedded.search(clip.embed_text("shop")))

drawing.to_svg("assets/embedding.svg")
```



![output](assets/embedding.svg)
