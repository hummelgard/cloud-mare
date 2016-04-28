from django.db import models
from django.utils.translation import ugettext_lazy as _
from django.contrib.auth.models import User
#from mezzanine.pages.models import Page

# Create your models here.

class HorseTracker(models.Model):
    #
    def __str__(self):
        return str(self.pk)+":"+ self.IMEI[-4:]+"-"+self.IMSI[-4:] 

    IMEI = models.CharField(_('IMEI number'), help_text=_('Trackers IMEI number'), max_length=20)
    IMSI = models.CharField(_('IMSI number'), help_text=_('Trackers IMSI number (sim-card)') 
,max_length=20)
    version = models.CharField(_('version'), help_text=_('revision number of tracker unit'), 
max_length=20)
    user = models.ForeignKey(User, verbose_name=_('user'))
    class Meta:
        verbose_name = _('horse tracker')
        verbose_name_plural = _('horse trackers')
        permissions = (
            ("tracker_user", "Can use tracker features"),           
        )

class Horse(models.Model):
    #
    def __str__(self):
        return self.name

    birthdate = models.DateTimeField(_('birth date'))
    name = models.CharField(_('name'), max_length=100)
    type = models.CharField(_('type'), max_length=50)
    gender= models.CharField(_('gender'), max_length=20)
    color = models.CharField(_('color'), max_length=50)
    user = models.ForeignKey(User, verbose_name=_('user'))
    tracker = models.ForeignKey(HorseTracker, verbose_name=_('horse tracker'), help_text=_('the cloudMARE tracker device used'))
    class Meta:
        verbose_name=_('horse')
        verbose_name_plural=_('horses')


class HorseData(models.Model):
    #
    def __str__(self):
        return str(self.date)

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
        verbose_name=_('horse data')
        verbose_name_plural=_('horse data')
        permissions = (
            ("tracker_user", "Can use tracker features"),           
        )
