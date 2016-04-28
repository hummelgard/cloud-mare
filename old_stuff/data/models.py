from django.db import models
from django.utils.translation import ugettext_lazy as _
from django.contrib.auth.models import User


# Create your models here.

class HorseTracker(models.Model):
    IMEI = models.CharField(_('IMEI number'), help_text=_('Trackers IMEI number') ,max_length=200)
    version = models.IntegerField(_('version'), help_text=_('revision number of tracker unit'), default=0)
    user = models.ForeignKey(User, verbose_name=_('user'))
    class Meta:
        verbose_name = _('horse tracker')
        verbose_name_plural = _('horse trackers')

class Horse(models.Model):
    birthdate = models.DateTimeField(_('birth date'))
    name = models.CharField(_('name'), max_length=200)
    type = models.CharField(_('type'), max_length=200)
    gender= models.CharField(_('gender'), max_length=200)
    color = models.CharField(_('color'), max_length=200)
    user = models.ForeignKey(User, verbose_name=_('user'))
    tracker = models.ForeignKey(HorseTracker, verbose_name=_('horse tracker'), help_text=_('the cloudMARE tracker device used'))
    class Meta:
        verbose_name=_('horse')
        verbose_name_plural=_('horses')

class HorsePositionData(models.Model):
    tracker = models.ForeignKey(HorseTracker, verbose_name=_('horse tracker'))
    date = models.DateTimeField(_('log date'))
    batteryVoltage = models.IntegerField(_('battery potential (mV)'), default=0)
    batteryCharge = models.IntegerField(_('battery charge (%)'), default=0)
    latitude = models.FloatField(_('latitude'), default=0)
    longitude = models.FloatField(_('longitude'), default=0)
    temperature = models.FloatField(_('air temperature'), default=0)
    humidity = models.FloatField(_('air humidity'), default=0)
    pressure = models.FloatField(_('air pressure'), default=0)
    MPUtemp = models.FloatField(_('MPU temperature'), default=0)
    accX = models.IntegerField(_('acceleration X'), default=0)
    accY = models.IntegerField(_('acceleration Y'), default=0)
    accZ = models.IntegerField(_('acceleration Z'), default=0)
    magX = models.IntegerField(_('mag. field X'), default=0)
    magY = models.IntegerField(_('mag. field Y'), default=0)
    magZ = models.IntegerField(_('mag. field Z'), default=0)
    class Meta:
        verbose_name=_('horse pos. data')
        verbose_name_plural=_('horses pos. data')
