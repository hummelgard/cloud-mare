from django.conf.urls import url
from . import views

urlpatterns = [
    url(r'^add/$', views.addData, name='add data'),
    url(r'^$', views.index, name='index'),
]

