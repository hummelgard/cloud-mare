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
                ('id', models.AutoField(verbose_name='ID', primary_key=True, auto_created=True, serialize=False)),
                ('birthdate', models.DateTimeField(verbose_name='birth date')),
                ('name', models.CharField(max_length=100, verbose_name='name')),
                ('type', models.CharField(max_length=50, verbose_name='type')),
                ('type_sv', models.CharField(max_length=50, null=True, verbose_name='type')),
                ('type_en', models.CharField(max_length=50, null=True, verbose_name='type')),
                ('gender', models.CharField(max_length=20, verbose_name='gender')),
                ('gender_sv', models.CharField(max_length=20, null=True, verbose_name='gender')),
                ('gender_en', models.CharField(max_length=20, null=True, verbose_name='gender')),
                ('color', models.CharField(max_length=50, verbose_name='color')),
                ('color_sv', models.CharField(max_length=50, null=True, verbose_name='color')),
                ('color_en', models.CharField(max_length=50, null=True, verbose_name='color')),
            ],
            options={
                'verbose_name_plural': 'horses',
                'verbose_name': 'horse',
            },
        ),
        migrations.CreateModel(
            name='HorseData',
            fields=[
                ('id', models.AutoField(verbose_name='ID', primary_key=True, auto_created=True, serialize=False)),
                ('date', models.DateTimeField(verbose_name='log date')),
                ('batteryVoltage', models.IntegerField(default=0, verbose_name='battery potential (mV)')),
                ('batteryCharge', models.IntegerField(default=0, verbose_name='battery charge (%)')),
                ('latitude', models.FloatField(default=0, verbose_name='latitude')),
                ('longitude', models.FloatField(default=0, verbose_name='longitude')),
                ('temperature', models.FloatField(default=0, verbose_name='air temperature')),
                ('humidity', models.FloatField(default=0, verbose_name='air humidity')),
                ('pressure', models.FloatField(default=0, verbose_name='air pressure')),
                ('MPUtemp', models.FloatField(default=0, verbose_name='MPU temperature')),
                ('accX', models.IntegerField(default=0, verbose_name='acceleration X')),
                ('accY', models.IntegerField(default=0, verbose_name='acceleration Y')),
                ('accZ', models.IntegerField(default=0, verbose_name='acceleration Z')),
                ('magX', models.IntegerField(default=0, verbose_name='mag. field X')),
                ('magY', models.IntegerField(default=0, verbose_name='mag. field Y')),
                ('magZ', models.IntegerField(default=0, verbose_name='mag. field Z')),
            ],
            options={
                'verbose_name_plural': 'horse data',
                'permissions': (('tracker_user', 'Can use tracker features'),),
                'verbose_name': 'horse data',
            },
        ),
        migrations.CreateModel(
            name='HorseTracker',
            fields=[
                ('id', models.AutoField(verbose_name='ID', primary_key=True, auto_created=True, serialize=False)),
                ('IMEI', models.CharField(max_length=20, verbose_name='IMEI number', help_text='Trackers IMEI number')),
                ('IMSI', models.CharField(max_length=20, verbose_name='IMSI number', help_text='Trackers IMSI number (sim-card)')),
                ('version', models.CharField(max_length=20, verbose_name='version', help_text='revision number of tracker unit')),
                ('user', models.ForeignKey(to=settings.AUTH_USER_MODEL, verbose_name='user')),
            ],
            options={
                'verbose_name_plural': 'horse trackers',
                'permissions': (('tracker_user', 'Can use tracker features'),),
                'verbose_name': 'horse tracker',
            },
        ),
        migrations.AddField(
            model_name='horsedata',
            name='tracker',
            field=models.ForeignKey(to='data.HorseTracker', verbose_name='horse tracker'),
        ),
        migrations.AddField(
            model_name='horse',
            name='tracker',
            field=models.ForeignKey(verbose_name='horse tracker', help_text='the cloudMARE tracker device used', to='data.HorseTracker'),
        ),
        migrations.AddField(
            model_name='horse',
            name='user',
            field=models.ForeignKey(to=settings.AUTH_USER_MODEL, verbose_name='user'),
        ),
    ]
