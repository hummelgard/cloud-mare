from modeltranslation.translator import translator, TranslationOptions
from .models import Horse

class HorseTranslationOptions(TranslationOptions):
    fields = ('type', 'gender','color',)


translator.register(Horse, HorseTranslationOptions)
