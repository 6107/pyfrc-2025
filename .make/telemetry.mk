# ------------------------------------------------------------------------ #
#      o-o      o                o                                         #
#     /         |                |                                         #
#    O     o  o O-o  o-o o-o     |  oo o--o o-o o-o                        #
#     \    |  | |  | |-' |   \   o | | |  |  /   /                         #
#      o-o o--O o-o  o-o o    o-o  o-o-o--O o-o o-o                        #
#             |                           |                                #
#          o--o                        o--o                                #
#                        o--o      o         o                             #
#                        |   |     |         |  o                          #
#                        O-Oo  o-o O-o  o-o -o-    o-o o-o                 #
#                        |  \  | | |  | | |  |  | |     \                  #
#                        o   o o-o o-o  o-o  o  |  o-o o-o                 #
#                                                                          #
#    Jemison High School - Huntsville Alabama                              #
# ------------------------------------------------------------------------ #
#
# OpenTelemetry setup and support.
#
#   Since this contains both configuration and targets, place it at some point
#   after your default target, not necessarily near the top top of the main
#   Makefile
#
JAEGER_VERSION  ?= "jaegertracing/all-in-one:1.48.0"            # Jaeger All-in-one container
GRAFANA_VERSION ?= "grafana/grafana-enterprise:10.1.0-ubuntu"   # Grafana container

# Uncomment the following (and modify) if you wish to have persistent storage of Grafana
# data. If not populated, all data will be cleaned up/removed once the Grafana container
# is shut down
# GRAFANA_PERSISTENT_VOLUME ?= "/opt/grafana"

ifdef GRAFANA_PERSISTENT_VOLUME
	GRAFANA_VOLUME_COMMAND := "echo 'Persistent storage of Grafana data is disabled'"
	GRAFANA_VOLUME_OPTIONS := ""
else
	GRAFANA_VOLUME_COMMAND := "docker volume create grafana-metrics-storage"
	GRAFANA_VOLUME_OPTIONS := "--volume grafana-metrics-storage:$(GRAFANA_PERSISTENT_VOLUME)"
endif

# User accessible UI ports for telemetry information
JAEGER_UI  ?= 16686
GRAFANA_UI ?= 3000

.PHONY: venv-telemetry jaeger-start jaeger-stop grafana-start grafana-stop telemetry-start telemetry-stop

## OpenTelemetry Support
venv-telemetry: $(PACKAGE_DIR)/telemetry-requirements.txt  ## Application virtual environment with OpenTelemetry support
	$(Q) make REQUIREMENTS=$(PACKAGE_DIR)/telemetry-requirements.txt venv

######################################################################
#  Port	Proto	Component	Function
#  6831	UDP		agent		accept jaeger.thrift over Thrift-compact protocol (used by most SDKs)
#  6832	UDP		agent		accept jaeger.thrift over Thrift-binary protocol (used by Node.js SDK)
#  5775	UDP		agent		(deprecated) accept zipkin.thrift over compact Thrift protocol (used by legacy clients only)
#  5778	HTTP	agent		serve configs (sampling, etc.)
# 16686	HTTP	query		serve frontend
#  4317	HTTP	collector	accept OpenTelemetry Protocol (OTLP) over gRPC, if enabled
#  4318	HTTP	collector	accept OpenTelemetry Protocol (OTLP) over HTTP, if enabled
# 14268	HTTP	collector	accept jaeger.thrift directly from clients
# 14250	HTTP	collector	accept model.proto
#  9411	HTTP	collector	Zipkin compatible endpoint (optional)
#
jaeger-start:      ## Start the Jaeger All-In-One container
	$ echo "Starting Jaeger All-in-one version ${JAEGER_VERSION}"
	docker run -d --name jaeger-all-in-one \
	  -e COLLECTOR_ZIPKIN_HTTP_PORT=:9411 \
	  -e COLLECTOR_OTLP_ENABLED=true \
	  -p 6831:6831/udp \
	  -p 6832:6832/udp \
	  -p 5778:5778 \
	  -p 4317:4317 \
	  -p 4318:4318 \
	  -p 14250:14250 \
	  -p 14268:14268 \
	  -p 14269:14269 \
	  -p 9411:9411 \
	  -p ${JAEGER_UI}:16686 \
	  ${JAEGER_VERSION}
	@ echo "Navigate to localhost:${JAEGER_UI} to access the Jaeger UI"

jaeger-stop:           ## Stop and remove the Jaeger All-In-One container
	$ echo "Stopping Jaeger All-in-one"
	- docker stop jaeger-all-in-one
	- docker rm jaeger-all-in-one

######################################################################
#  Port	Proto	Component	Function
#  6831	UDP		agent		accept jaeger.thrift over Thrift-compact protocol (used by most SDKs)
#
grafana-start:         ## Start the Grafana container for metrics support
	$ echo "Starting Grafana version ${GRAFANA_VERSION}"
	$(GRAFANA_VOLUME_COMMAND)
	docker run -d --name grafana-metrics-p ${GRAFANA_UI}:3000/udp ${GRAFANA_VOLUME_OPTIONS} ${GRAFANA_VERSION}
	@ echo "Navigate to localhost:${GRAFANA_UI} to access the Grafana UI"

grafana-stop:          ## Stop and remove the Grafana container
	$ echo "Stopping Grafana metrics container"
	- docker stop grafana-metrics
	- docker rm grafana-metrics

telemetry-start: jaeger-start grafana-start   ## Start all OpenTelemetry support containers
	@ echo "Telemetry containers started"

telemetry-stop: jaeger-stop grafana-stop      ## Stop and remove all OpenTelemetry support containers
	@ echo "Telemetry containers stopped and removed"
