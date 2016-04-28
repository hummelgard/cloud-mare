# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import models, migrations


class Migration(migrations.Migration):

    dependencies = [
        ('data', '0008_auto_20160428_1224'),
    ]

    operations = [
        migrations.AlterField(
            model_name='horse',
            name='tracker',
            field=models.ForeignKey(blank=True, default='', help_text='the cloudMARE tracker device used', verbose_name='horse tracker', to='data.HorseTracker', null=True),
        ),
    ]
