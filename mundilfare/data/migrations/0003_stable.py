# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import models, migrations


class Migration(migrations.Migration):

    dependencies = [
        ('pages', '0004_auto_20151027_1542'),
        ('data', '0002_horsedata_irtemp'),
    ]

    operations = [
        migrations.CreateModel(
            name='Stable',
            fields=[
                ('page_ptr', models.OneToOneField(auto_created=True, to='pages.Page', primary_key=True, serialize=False, parent_link=True)),
                ('stablename', models.CharField(verbose_name='stable name', max_length=100)),
                ('stablename_sv', models.CharField(verbose_name='stable name', null=True, max_length=100)),
                ('stablename_en', models.CharField(verbose_name='stable name', null=True, max_length=100)),
            ],
            options={
                'verbose_name': 'stable',
                'ordering': ('_order',),
                'verbose_name_plural': 'stables',
            },
            bases=('pages.page',),
        ),
    ]
