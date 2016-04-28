# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import models, migrations


class Migration(migrations.Migration):

    dependencies = [
        ('data', '0002_auto_20151024_1925'),
    ]

    operations = [
        migrations.AlterField(
            model_name='horsetracker',
            name='version',
            field=models.CharField(verbose_name='version', help_text='revision number of tracker unit', max_length=20),
        ),
    ]
