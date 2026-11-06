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

## Screen bench, image set v2 (5 Nov 2026)

`screen_bench.py run --stimuli v2` uses six more flames from Wikimedia Commons, downloaded on 2026-11-05, with `candle_and_match` and `candle_blow` above.
Two more files gave the error 429 (too many requests) and are not here: `Corona.Candle.webm` (CC0) and `Water candle experiment.webm` (CC BY 3.0).

| File name used here | Commons file | Author | Licence | Content |
|---|---|---|---|---|
| `blue_candle_Jahobr_CC0.webm` | [Blue candle burning.webm](https://commons.wikimedia.org/wiki/File:Blue_candle_burning.webm) | Jahobr | CC0 | Blue candle, close, dark room |
| `pink_candle_Jahobr_CC0.webm` | [Pink candle burning.webm](https://commons.wikimedia.org/wiki/File:Pink_candle_burning.webm) | Jahobr | CC0 | Pink candle, close, dark room |
| `white_candle_Jahobr_CC0.webm` | [White candle video.webm](https://commons.wikimedia.org/wiki/File:White_candle_video.webm) | Jahobr | CC0 | White candle, close, dark room |
| `candle_blown_out_CCBYSA3.ogv` | [CandleGettingBlownOut.ogv](https://commons.wikimedia.org/wiki/File:CandleGettingBlownOut.ogv) | Peter Michner | CC BY-SA 3.0 | Tall candle, then blown out |
| `candle_shadow_Rhetos_CCBYSA4.webm` | [Kerzen-Schatten-Versuch.webm](https://commons.wikimedia.org/wiki/File:Kerzen-Schatten-Versuch.webm) | Rhetos | CC BY-SA 4.0 | Small candle and a figurine, shadow experiment, dark room |
| `candle_suffocated_CCBYSA3.ogv` | [ErstickteKerze.ogv](https://commons.wikimedia.org/wiki/File:ErstickteKerze.ogv) | LoKiLeCh | CC BY-SA 3.0 | Tea light on a plate, a glass puts it out |
| `candle_flame_myanmar_CC0.webm` | [ဖယောင်းတိုင်မီးတောက်.webm](https://commons.wikimedia.org/wiki/File:ဖယောင်းတိုင်မီးတောက်.webm) | စာကလေ | CC0 | Candle flame. Not used: the detector box changes from 28 to 250 px |
| `candle_smoke_ignition_Rhetos_CCBYSA4.webm` | [Rauchfaden-Zünd-Versuch.webm](https://commons.wikimedia.org/wiki/File:Rauchfaden-Zünd-Versuch.webm) | Rhetos | CC BY-SA 4.0 | Candle lit again through its smoke. Not used: a flame in 1 of 24 frames |
| `wax_burner_Rhetos_CCBYSA4.webm` | [Wachsbrenner (minimalistisch).webm](https://commons.wikimedia.org/wiki/File:Wachsbrenner_(minimalistisch).webm) | Rhetos | CC BY-SA 4.0 | Wax burner. Not used: text over the image, not a candle |
| `candle_capacitor_PD.ogv` | [Kerzenflamme im plattenkondensator.ogv](https://commons.wikimedia.org/wiki/File:Kerzenflamme_im_plattenkondensator.ogv) | (see Commons) | Public domain | Candle flame between two plates. Not used: reflections give 3 boxes |

To download the six files of the set, run these commands one at a time. Wait a few seconds between them: Wikimedia answers 429 to fast requests.

```sh
curl -L --create-dirs -A "firefighter-robot-student-project" -o ~/datasets/firefighter/videos/blue_candle_Jahobr_CC0.webm "https://upload.wikimedia.org/wikipedia/commons/4/4f/Blue_candle_burning.webm"
```

```sh
curl -L --create-dirs -A "firefighter-robot-student-project" -o ~/datasets/firefighter/videos/pink_candle_Jahobr_CC0.webm "https://upload.wikimedia.org/wikipedia/commons/6/65/Pink_candle_burning.webm"
```

```sh
curl -L --create-dirs -A "firefighter-robot-student-project" -o ~/datasets/firefighter/videos/white_candle_Jahobr_CC0.webm "https://upload.wikimedia.org/wikipedia/commons/1/17/White_candle_video.webm"
```

```sh
curl -L --create-dirs -A "firefighter-robot-student-project" -o ~/datasets/firefighter/videos/candle_blown_out_CCBYSA3.ogv "https://upload.wikimedia.org/wikipedia/commons/1/11/CandleGettingBlownOut.ogv"
```

```sh
curl -L --create-dirs -A "firefighter-robot-student-project" -o ~/datasets/firefighter/videos/candle_shadow_Rhetos_CCBYSA4.webm "https://upload.wikimedia.org/wikipedia/commons/d/d1/Kerzen-Schatten-Versuch.webm"
```

```sh
curl -L --create-dirs -A "firefighter-robot-student-project" -o ~/datasets/firefighter/videos/candle_suffocated_CCBYSA3.ogv "https://upload.wikimedia.org/wikipedia/commons/1/14/ErstickteKerze.ogv"
```
