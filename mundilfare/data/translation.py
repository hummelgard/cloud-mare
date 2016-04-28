from modeltranslation.translator import translator, TranslationOptions
from .models import Stable
from .models import Horse

class StableTranslationOptions(TranslationOptions):
    fields = ('stablename',)



#class HorseTranslationOptions(TranslationOptions):
#    fields = ('type', 'gender','color',)


translator.register(Stable, StableTranslationOptions)
#translator.register(Horse, HorseTranslationOptions)
