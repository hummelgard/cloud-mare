# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import models, migrations


class Migration(migrations.Migration):

    dependencies = [
        ('pages', '0004_auto_20151027_1542'),
        ('data', '0004_data'),
    ]

    operations = [
        migrations.RemoveField(
            model_name='data',
            name='page_ptr',
        ),
        migrations.DeleteModel(
            name='data',
        ),
    ]
