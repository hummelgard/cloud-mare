# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import models, migrations


class Migration(migrations.Migration):

    dependencies = [
        ('data', '0009_auto_20160428_1225'),
    ]

    operations = [
        migrations.AlterField(
            model_name='horse',
            name='tracker',
            field=models.ForeignKey(help_text='the mundilfare tracker device used', default='', to='data.HorseTracker', null=True, verbose_name='horse tracker', blank=True),
        ),
    ]
