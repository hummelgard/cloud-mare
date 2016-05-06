from django.forms import ModelMultipleChoiceField

from django.utils import timezone
import datetime, pytz
from tzlocal import get_localzone


class HorsedataModelMultipleChoiceField(ModelMultipleChoiceField):
    def label_from_instance(self, horsedata):

        local_tz = get_localzone()
        
        datetime_UTC = horsedata.date.replace(tzinfo=pytz.timezone('UTC'))
 
        datetime_LOCAL = datetime_UTC.astimezone(local_tz)
        return "{0} - {1}- {2}".format( str(datetime_LOCAL)[:-6], 
                                        horsedata.temperature, 
                                   horsedata.activity()
                                 )

