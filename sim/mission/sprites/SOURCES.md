# Real-fire flame sprites: sources and licences

`make_sprites.py` (one folder up) makes these files from videos on Wikimedia Commons. We downloaded the videos on 2026-10-15.
The script downloads them again when they are missing, and checks the SHA-1 of each file.

| Sprite file | Commons file | Author | Licence | Content | Start (s) | Crop box in the video (x0, y0, x1, y1, px) | Horizontal squeeze |
|---|---|---|---|---|---|---|---|
| `free_flame.webp` | [Fire.ogv](https://commons.wikimedia.org/wiki/File:Fire.ogv) | Cary Bass (User:Bastique) | Public domain (PD-self) | A campfire flame on a black background, 480 x 640 | 1.0 | 60, 10, 390, 510 | 0.76 |
| `basket.webp` | [Feuer.webm](https://commons.wikimedia.org/wiki/File:Feuer.webm) | Brützlschnätz | CC BY-SA 4.0 | A small fire in a fire basket at night, 1920 x 1080. We cut the flames above the basket | 2.0 | 760, -60, 1200, 680 | 0.84 |
| `campfire_a.webp` | [Campfire Video 1.webm](https://commons.wikimedia.org/wiki/File:Campfire_Video_1.webm) | Virginia State Parks (Flickr) | CC BY 2.0 | A small campfire on the ground at night, 608 x 1080. The flames are overexposed | 2.0 | 175, 300, 435, 620 | 0.62 |
| `campfire_b.webp` | [Campfire video 2.webm](https://commons.wikimedia.org/wiki/File:Campfire_video_2.webm) | Virginia State Parks (Flickr) | CC BY 2.0 | A small wood fire in a metal box at night, 608 x 1080. The flames are overexposed | 2.0 | 170, 50, 470, 530 | 0.80 |
| `wood_fire.webp` | [Fire burning.ogv](https://commons.wikimedia.org/wiki/File:Fire_burning.ogv) | Zouavman Le Zouave | CC BY-SA 4.0 | A wood fire at night, 640 x 480. We cut a slice from the middle | 1.0 | 180, 0, 460, 470 | 0.84 |

**Not verified:** The two Virginia State Parks files carry the Commons tag "LicenseReview": nobody has checked the Flickr licence yet (2026-10-15).

## Changes to the videos

For each clip, `make_sprites.py` does these steps:

1. It takes 2.4 s of the video from the start time and samples it at 30 frames per second.
2. It cuts the crop box. Parts of the box outside the video are black.
3. It scales the box to 128 x 256 pixels. This squeezes the flames horizontally to the billboard shape of the fire model (`world.flame_size`).
4. It makes the alpha channel from the brightness (the largest of R, G and B, between a black level and a white level).
   Grey and blue pixels lose alpha, so smoke and lit ground do not count as flame. The white flame core keeps its alpha.
5. It fades the alpha to 0 at the left, right and top edges of the box.
6. It fades the last 0.4 s into the first frames, so the 2 s clip loops without a jump.
7. It saves the 60 frames in a 10 x 6 grid as one WebP image (lossy colour, quality 92, lossless alpha).

The simulation mirrors the clip horizontally for every other fire. It also scales and blends the clip into the camera image, adds a glow, and adds the camera noise.

## Licence of the sprites and of images made with them

- Each sprite file is a derived work of its video. It keeps the licence of the video: public domain, CC BY 2.0 or CC BY-SA 4.0.
- Give the attribution above when you share a sprite file.
- A camera image or a video that shows `basket` or `wood_fire` is also a derived work of a CC BY-SA 4.0 video. Share it under CC BY-SA 4.0, with the attributions above.

## Clips we did not use

- Candle flames (for example "Candle and Match.ogv"): a candle flame is smooth and laminar. A 10 to 20 cm tray fire is turbulent. A candle flame scaled to 25 to 50 cm would not look like a tray fire.
- "Burning PET bottle.ogv" (CC0): the flames are overexposed into one large white area with smoke. The crop gave a white block, not a flame.
- "Kaminfeuer.webm" and the "Fire آتش" series: the background is a lit fireplace wall or green forest. Brightness and colour cannot separate the flames from it.
