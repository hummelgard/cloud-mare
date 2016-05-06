from django.conf.urls import url
from . import views

app_name = 'data'
urlpatterns = [
    url(r'^add/$', views.tracker_add_data, name='tracker add data'),

    url(r'^horses/$', views.horse_list, name='horses list'),
    url(r'^horses/(?P<name>[-\w\d]+)-(?P<id>\d+)/$', views.horse_detail, name='horse detail'),

    url(r'^horsetrackers/$', views.HorsetrackerView.as_view(), name='horsetrackers list'),
    #url(r'^horsetrackers/$', views.horsetracker_list, name='horsetrackers list'),
    url(r'^horsetrackers/(?P<trackerID>[-\w\d:]+)/$', views.horsetracker_detail, name='horsetracker detail'),

    url(r'^map/(?P<trackerID>[-\w\d:]+)/$', views.googlemap_intensity, name='google map intensity'),


    url(r'^ajax/horsetrackerlist/(?P<trackerID>[-\w\d:]+)/$', 
        views.horsedata_list_asJson, 
        name='horsedata list asJson'),

    url(r'^horsedata/(?P<trackerID>[-\w\d:]+)/$', 
        views.HorsedataListView.as_view( 
        paginate_by=10),
        name='horsedata list'),
]

