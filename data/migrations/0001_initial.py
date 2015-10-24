# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import models, migrations
from django.conf import settings


class Migration(migrations.Migration):

    dependencies = [
        migrations.swappable_dependency(settings.AUTH_USER_MODEL),
    ]

    operations = [
        migrations.CreateModel(
            name='Horse',
            fields=[
                ('id', models.AutoField(auto_created=True, serialize=False, primary_key=True, verbose_name='ID')),
                ('birthdate', models.DateTimeField(verbose_name='birth date')),
                ('name', models.CharField(verbose_name='name', max_length=200)),
                ('type', models.CharField(verbose_name='type', max_length=200)),
                ('gender', models.CharField(verbose_name='gender', max_length=200)),
                ('color', models.CharField(verbose_name='color', max_length=200)),
            ],
            options={
                'verbose_name': 'horse',
                'verbose_name_plural': 'horses',
            },
        ),
        migrations.CreateModel(
            name='HorsePositionData',
            fields=[
                ('id', models.AutoField(auto_created=True, serialize=False, primary_key=True, verbose_name='ID')),
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
                ('horse', models.ForeignKey(verbose_name='horse', to='data.Horse')),
            ],
            options={
                'verbose_name': 'horse pos. data',
                'verbose_name_plural': 'horses pos. data',
            },
        ),
        migrations.CreateModel(
            name='HorseTracker',
            fields=[
                ('id', models.AutoField(auto_created=True, serialize=False, primary_key=True, verbose_name='ID')),
                ('IMEI', models.CharField(help_text='Trackers IMEI number', verbose_name='IMEI number', max_length=200)),
                ('version', models.IntegerField(help_text='revision number of tracker unit', verbose_name='version', default=0)),
                ('user', models.ForeignKey(verbose_name='user', to=settings.AUTH_USER_MODEL)),
            ],
            options={
                'verbose_name': 'horse tracker',
                'verbose_name_plural': 'horse trackers',
            },
        ),
        migrations.AddField(
            model_name='horse',
            name='tracker',
            field=models.ForeignKey(help_text='the cloudMARE tracker device used', verbose_name='horse tracker', to='data.HorseTracker'),
        ),
        migrations.AddField(
            model_name='horse',
            name='user',
            field=models.ForeignKey(verbose_name='user', to=settings.AUTH_USER_MODEL),
        ),
    ]
