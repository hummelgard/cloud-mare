# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import models, migrations


class Migration(migrations.Migration):

    dependencies = [
        ('data', '0004_auto_20160428_1203'),
    ]

    operations = [
        migrations.RemoveField(
            model_name='horse',
            name='color_en',
        ),
        migrations.RemoveField(
            model_name='horse',
            name='color_sv',
        ),
        migrations.RemoveField(
            model_name='horse',
            name='gender_en',
        ),
        migrations.RemoveField(
            model_name='horse',
            name='gender_sv',
        ),
        migrations.RemoveField(
            model_name='horse',
            name='type_en',
        ),
        migrations.RemoveField(
            model_name='horse',
            name='type_sv',
        ),
    ]
