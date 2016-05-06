from django import forms
from django.contrib.auth.models import User
from .models import Horse, HorseTracker, HorseData
from .fields import HorsedataModelMultipleChoiceField

class SelectHorsedataForm(forms.Form):

    def __init__(self, queryset, *args, **kwargs):
        super(SelectHorsedataForm, self).__init__(*args, **kwargs)

        name = forms.CharField()
        message = forms.CharField(widget=forms.Textarea)
        #horsedata = HorseData.objects.all().order_by('-date')
        self.fields['formdata'] = HorsedataModelMultipleChoiceField(queryset=queryset,
to_field_name="temperature", widget=forms.SelectMultiple(attrs={'cols': '60','size':'40'}))
        #formdata = forms.ModelMultipleChoiceField(queryset=queryset)

    def send_email(self):
        # send email using the self.cleaned_data dictionary
        pass


