# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import models, migrations


class Migration(migrations.Migration):

    dependencies = [
        ('data', '0007_auto_20160428_1217'),
    ]

    operations = [
        migrations.AlterField(
            model_name='horse',
            name='tracker',
            field=models.ForeignKey(to='data.HorseTracker', blank=True, help_text='the cloudMARE tracker device used', verbose_name='horse tracker', default=''),
        ),
    ]
