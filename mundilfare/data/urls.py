from django.conf.urls import url
from . import views

app_name = 'data'
urlpatterns = [

################ TRACKER ADD DATA
    url(r'^add/$', views.tracker_add_data, name='tracker add data'),


################ HORSES
    url(r'^horses/$', views.horse_list, name='horses list'),
    url(r'^horses/(?P<name>[-\w\d]+)-(?P<id>\d+)/$', views.horse_detail, name='horse detail'),


################ HORSETRACKERS
    url(r'^horsetrackers/$', views.HorsetrackerView.as_view(), name='horsetrackers list'),
    #url(r'^horsetrackers/$', views.horsetracker_list, name='horsetrackers list'),
    url(r'^horsetrackers/(?P<trackerID>[-\w\d:]+)/$', views.horsetracker_detail, name='horsetracker detail'),

################ HORSEDATA
    url(r'^horsedata/(?P<trackerID>[-\w\d:]+)/$', 
        views.HorsedataListView.as_view( 
        paginate_by=100),
        name='horsedata list'),

################ ANALYSIS
    url(r'^mapI/(?P<trackerID>[-\w\d:]+)/$', views.googlemap_intensity, name='google map intensity'),
    url(r'^mapP/(?P<trackerID>[-\w\d:]+)/$', views.googlemap_position, name='google map position'),

    url(r'^analysis/(?P<trackerID>[-\w\d:]+)/$', views.AnalysisIndexView.as_view(), name='analysis index'),





################ AJAX STUFF

    url(r'^ajax/horsetrackerlist/(?P<trackerID>[-\w\d:]+)/$', 
        views.horsedata_list_asJson, 
        name='horsedata list asJson'),


]

