# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import models, migrations


class Migration(migrations.Migration):

    dependencies = [
        ('data', '0001_initial'),
    ]

    operations = [
        migrations.AddField(
            model_name='horsetracker',
            name='IMSI',
            field=models.CharField(default=0, verbose_name='IMSI number', max_length=20, help_text='Trackers IMSI number (sim-card)'),
            preserve_default=False,
        ),
        migrations.AlterField(
            model_name='horse',
            name='color',
            field=models.CharField(verbose_name='color', max_length=50),
        ),
        migrations.AlterField(
            model_name='horse',
            name='gender',
            field=models.CharField(verbose_name='gender', max_length=20),
        ),
        migrations.AlterField(
            model_name='horse',
            name='name',
            field=models.CharField(verbose_name='name', max_length=100),
        ),
        migrations.AlterField(
            model_name='horse',
            name='type',
            field=models.CharField(verbose_name='type', max_length=50),
        ),
        migrations.AlterField(
            model_name='horsetracker',
            name='IMEI',
            field=models.CharField(help_text='Trackers IMEI number', verbose_name='IMEI number', max_length=20),
        ),
    ]
