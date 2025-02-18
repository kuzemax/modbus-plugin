将libmodbus_plugin.so放入/opt/arkime/plugins中

编辑/opt/arkime/etc/config.ini

加上

```
[default]
# Comma seperated list of OpenSearch/Elasticsearch host:port combinations. If not using a
# Elasticsearch load balancer, a different OpenSearch/Elasticsearch node in the cluster can be
# specified for each Arkime node to help spread load on high volume clusters. For user/password
# use https://user:pass@host:port OR elasticsearchBasicAuth
elasticsearch=http://localhost:9200
# elasticsearchBasicAuth=
plugins=libmodbus_plugin.so
```

