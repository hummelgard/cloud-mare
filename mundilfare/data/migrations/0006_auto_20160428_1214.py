# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import models, migrations


class Migration(migrations.Migration):

    dependencies = [
        ('data', '0005_auto_20160428_1204'),
    ]

    operations = [
        migrations.AlterField(
            model_name='horse',
            name='tracker',
            field=models.ForeignKey(verbose_name='horse tracker', default='---', help_text='the cloudMARE tracker device used', to='data.HorseTracker'),
        ),
    ]
