# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES )
set(COMPONENT_PRIV_REQUIRES esp32-mavlink nvs_flash bt)

set(COMPONENT_SRCS "main.c" "server.c")
set(COMPONENT_ADD_INCLUDEDIRS ".")

register_component()
