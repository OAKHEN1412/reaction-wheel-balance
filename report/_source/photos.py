"""Compose the prototype photo pairs used in chapters 3-4.

    py -X utf8 photos.py <folder with the iPhone photos>

Writes figs/f_proto.jpg (figure 4.1: IMG_5799 + IMG_5790) and
figs/f_ch3_wiring.jpg (figure 3.13: IMG_5796 + IMG_5794). The photos were
sent from the phone over Taildrop on 2026-10-13 and are not kept in the repo.
"""
import os
import sys
from PIL import Image, ImageOps, ImageDraw, ImageFont

src = sys.argv[1]
os.makedirs('figs', exist_ok=True)
font = ImageFont.truetype(os.path.join(os.environ['LOCALAPPDATA'], 'Microsoft', 'Windows', 'Fonts',
                                       'THSarabunNew.ttf'), 64)


def load(name, crop_bottom=0.0):
    im = ImageOps.exif_transpose(Image.open(os.path.join(src, name))).convert('RGB')
    if crop_bottom:
        im = im.crop((0, 0, im.width, round(im.height * (1 - crop_bottom))))
    return im


def pair(ims, name, height, gap=30, label_h=90):
    ims = [im.resize((round(im.width * height / im.height), height), Image.LANCZOS) for im in ims]
    sheet = Image.new('RGB', (sum(im.width for im in ims) + gap, height + label_h), 'white')
    d = ImageDraw.Draw(sheet)
    x = 0
    for im, lab in zip(ims, ('(ก)', '(ข)')):
        sheet.paste(im, (x, 0))
        d.text((x + (im.width - d.textlength(lab, font=font)) / 2, height + 8), lab, fill='black', font=font)
        x += im.width + gap
    sheet.save(os.path.join('figs', name), quality=88)
    print('saved', name, sheet.size)


# IMG_5799's bottom edge shows the photographer's leg; trim it.
pair([load('IMG_5799.jpeg', crop_bottom=0.13), load('IMG_5790.jpeg')], 'f_proto.jpg', 1100)
pair([load('IMG_5796.jpeg'), load('IMG_5794.jpeg')], 'f_ch3_wiring.jpg', 900)
