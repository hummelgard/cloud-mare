# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import models, migrations


class Migration(migrations.Migration):

    dependencies = [
        ('pages', '0004_auto_20151027_1542'),
        ('data', '0003_stable'),
    ]

    operations = [
        migrations.CreateModel(
            name='data',
            fields=[
                ('page_ptr', models.OneToOneField(serialize=False, auto_created=True, primary_key=True, to='pages.Page', parent_link=True)),
            ],
            options={
                'ordering': ('_order',),
            },
            bases=('pages.page',),
        ),
    ]
