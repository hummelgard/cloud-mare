# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import models, migrations


class Migration(migrations.Migration):

    dependencies = [
        ('data', '0001_initial'),
    ]

    operations = [
        migrations.AddField(
            model_name='horsedata',
            name='IRtemp',
            field=models.FloatField(default=0, verbose_name='IR temperature'),
        ),
    ]
