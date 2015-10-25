# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import models, migrations


class Migration(migrations.Migration):

    dependencies = [
        ('data', '0003_auto_20151024_1928'),
    ]

    operations = [
        migrations.CreateModel(
            name='HorseData',
            fields=[
                ('id', models.AutoField(verbose_name='ID', serialize=False, auto_created=True, primary_key=True)),
                ('date', models.DateTimeField(verbose_name='log date')),
                ('batteryVoltage', models.IntegerField(verbose_name='battery potential (mV)', default=0)),
                ('batteryCharge', models.IntegerField(verbose_name='battery charge (%)', default=0)),
                ('latitude', models.FloatField(verbose_name='latitude', default=0)),
                ('longitude', models.FloatField(verbose_name='longitude', default=0)),
                ('temperature', models.FloatField(verbose_name='air temperature', default=0)),
                ('humidity', models.FloatField(verbose_name='air humidity', default=0)),
                ('pressure', models.FloatField(verbose_name='air pressure', default=0)),
                ('MPUtemp', models.FloatField(verbose_name='MPU temperature', default=0)),
                ('accX', models.IntegerField(verbose_name='acceleration X', default=0)),
                ('accY', models.IntegerField(verbose_name='acceleration Y', default=0)),
                ('accZ', models.IntegerField(verbose_name='acceleration Z', default=0)),
                ('magX', models.IntegerField(verbose_name='mag. field X', default=0)),
                ('magY', models.IntegerField(verbose_name='mag. field Y', default=0)),
                ('magZ', models.IntegerField(verbose_name='mag. field Z', default=0)),
                ('tracker', models.ForeignKey(verbose_name='horse tracker', to='data.HorseTracker')),
            ],
            options={
                'verbose_name': 'horse data',
                'verbose_name_plural': 'horse data',
            },
        ),
        migrations.RemoveField(
            model_name='horsepositiondata',
            name='tracker',
        ),
        migrations.DeleteModel(
            name='HorsePositionData',
        ),
    ]
