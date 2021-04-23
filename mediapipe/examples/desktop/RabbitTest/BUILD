licenses(["notice"])

package(default_visibility = ["//mediapipe/examples:__subpackages__"])

# Demo to receive RabbitMQ
cc_binary(
    name = "RabbitReceive",
    srcs = ["RabbitReceive.cc"],
    deps = [
        "@rabbitmq//:rabbitmq",
        "@boost//:boost",
    ],
)

# Demo to send RabbitMQ
cc_binary(
    name = "RabbitSend",
    srcs = ["RabbitSend.cc"],
    deps = [
        "@rabbitmq//:rabbitmq",
    ],
)

# Demo to parse json by Boost
cc_binary(
    name = "BoostParsingJson",
    srcs = ["BoostParsingJson.cc"],
    deps = [
        "@boost//:boost",
    ],
)

