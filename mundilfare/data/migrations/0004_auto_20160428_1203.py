# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import models, migrations


class Migration(migrations.Migration):

    dependencies = [
        ('data', '0003_stable'),
    ]

    operations = [
        migrations.AlterField(
            model_name='horse',
            name='color',
            field=models.CharField(default='', verbose_name='color', max_length=50),
        ),
        migrations.AlterField(
            model_name='horse',
            name='color_en',
            field=models.CharField(null=True, default='', verbose_name='color', max_length=50),
        ),
        migrations.AlterField(
            model_name='horse',
            name='color_sv',
            field=models.CharField(null=True, default='', verbose_name='color', max_length=50),
        ),
        migrations.AlterField(
            model_name='horse',
            name='gender',
            field=models.CharField(default='', verbose_name='gender', max_length=20),
        ),
        migrations.AlterField(
            model_name='horse',
            name='gender_en',
            field=models.CharField(null=True, default='', verbose_name='gender', max_length=20),
        ),
        migrations.AlterField(
            model_name='horse',
            name='gender_sv',
            field=models.CharField(null=True, default='', verbose_name='gender', max_length=20),
        ),
        migrations.AlterField(
            model_name='horse',
            name='type',
            field=models.CharField(default='', verbose_name='type', max_length=50),
        ),
        migrations.AlterField(
            model_name='horse',
            name='type_en',
            field=models.CharField(null=True, default='', verbose_name='type', max_length=50),
        ),
        migrations.AlterField(
            model_name='horse',
            name='type_sv',
            field=models.CharField(null=True, default='', verbose_name='type', max_length=50),
        ),
    ]
