from django import forms
from django.http import HttpResponseRedirect
from django.contrib.auth.models import User
from mezzanine.pages.page_processors import processor_for
from .models import Stable, HorseTracker, Horse

class AuthorForm(forms.Form):
    name = forms.CharField()
    email = forms.EmailField()

@processor_for("data/horsetrackers")
def author_form(request, page):
    current_user = request.user
    horsetrackers = HorseTracker.objects.filter(user=current_user)

    return {"horsetrackers": horsetrackers}

@processor_for("data/horses")
def author_form(request, page):
    current_user = request.user
    horses = Horse.objects.filter(user=current_user)
    
    return {"horses": horses}


