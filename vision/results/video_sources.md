# Real fire videos used by video_demo.py

All clips are from Wikimedia Commons. We downloaded them on 2026-10-15.
`video_demo.py` also writes `results/demo.mp4`: parts of these clips with the detector boxes. It is a derived work, so it carries the licence CC BY-SA 4.0 and the attributions below. It is not in the repository.

| File name used here | Commons file | Author | Licence | Content |
|---|---|---|---|---|
| `candle_and_match_LoKiLeCh_CCBYSA3.ogv` | [Candle and Match.ogv](https://commons.wikimedia.org/wiki/File:Candle_and_Match.ogv) | LoKiLeCh | CC BY-SA 3.0 | Candle, close, 640 x 480 |
| `candle_blow_GilesLaurent_CCBYSA4.webm` | [028 Lighting and blowing of a candle (normal speed) Video by Giles Laurent.webm](https://commons.wikimedia.org/wiki/File:028_Lighting_and_blowing_of_a_candle_(normal_speed)_Video_by_Giles_Laurent.webm) | Giles Laurent | CC BY-SA 4.0 | Tea light lit with a lighter, then blown out |
| `burning_pet_bottle_CC0.ogv` | [Burning PET bottle.ogv](https://commons.wikimedia.org/wiki/File:Burning_PET_bottle.ogv) | 01x07x2022000 | CC0 | A burning plastic bottle |
| `campfire_Vijayanrajapuram_CCBYSA4.webm` | [Campfire.webm](https://commons.wikimedia.org/wiki/File:Campfire.webm) | Vijayanrajapuram | CC BY-SA 4.0 | A small campfire, far away, with people around it |
| `flameless_candle_1_CC0.webm` | [Flameless candle 1.webm](https://commons.wikimedia.org/wiki/File:Flameless_candle_1.webm) | (CC0, derivative work on Commons) | CC0 | LED candle, no fire |
| `flameless_candle_2_CC0.webm` | [Flameless candle 2.webm](https://commons.wikimedia.org/wiki/File:Flameless_candle_2.webm) | (CC0, derivative work on Commons) | CC0 | LED candle, no fire |

To download them to `~/datasets/firefighter/videos/`, run these commands one at a time. Wikimedia asks for a user agent (`-A`):

```sh
curl -L --create-dirs -A "firefighter-robot-student-project" -o ~/datasets/firefighter/videos/candle_and_match_LoKiLeCh_CCBYSA3.ogv "https://upload.wikimedia.org/wikipedia/commons/6/6c/Candle_and_Match.ogv"
```

```sh
curl -L --create-dirs -A "firefighter-robot-student-project" -o ~/datasets/firefighter/videos/candle_blow_GilesLaurent_CCBYSA4.webm "https://upload.wikimedia.org/wikipedia/commons/a/a7/028_Lighting_and_blowing_of_a_candle_%28normal_speed%29_Video_by_Giles_Laurent.webm"
```

```sh
curl -L --create-dirs -A "firefighter-robot-student-project" -o ~/datasets/firefighter/videos/burning_pet_bottle_CC0.ogv "https://upload.wikimedia.org/wikipedia/commons/3/36/Burning_PET_bottle.ogv"
```

```sh
curl -L --create-dirs -A "firefighter-robot-student-project" -o ~/datasets/firefighter/videos/campfire_Vijayanrajapuram_CCBYSA4.webm "https://upload.wikimedia.org/wikipedia/commons/a/ad/Campfire.webm"
```

```sh
curl -L --create-dirs -A "firefighter-robot-student-project" -o ~/datasets/firefighter/videos/flameless_candle_1_CC0.webm "https://upload.wikimedia.org/wikipedia/commons/9/92/Flameless_candle_1.webm"
```

```sh
curl -L --create-dirs -A "firefighter-robot-student-project" -o ~/datasets/firefighter/videos/flameless_candle_2_CC0.webm "https://upload.wikimedia.org/wikipedia/commons/0/0b/Flameless_candle_2.webm"
```
