from django.forms import ModelMultipleChoiceField

from django.utils import timezone
import datetime, pytz
from tzlocal import get_localzone


class HorsedataModelMultipleChoiceField(ModelMultipleChoiceField):
    def label_from_instance(self, horsedata):

        local_tz = get_localzone()
        
        datetime_UTC = horsedata.date.replace(tzinfo=pytz.timezone('UTC'))
 
        datetime_LOCAL = datetime_UTC.astimezone(local_tz)
        datetime_str = datetime_LOCAL.strftime("%Y-%m-%d__%H:%M")
        return "{0} - Temp:{1:15.1f} - Aktvity:{2}".format( datetime_str, 
                                        horsedata.temperature, 
                                   horsedata.activity()
                                 )

