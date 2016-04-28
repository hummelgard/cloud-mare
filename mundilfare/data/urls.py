from django.conf.urls import url
from . import views


urlpatterns = [
    url(r'^add/$', views.AddData, name='add data'),
    url(r'^(?P<pk>[0-9]+)/$', views.HorseDataDetail, name='horse data detail'),
    url(r'^list$', views.HorseDataList, name='horse data list'),


    url(r'^index/$', views.IndexView.as_view(), name='index'),
    url(r'^horses/$', "mezzanine.pages.views.page", {"slug": "horses"}, name="horses"),
    url(r'^horsetrackers/$', "mezzanine.pages.views.page", {"slug": "horsetrackers"}, name="horsetrackers"),


]

