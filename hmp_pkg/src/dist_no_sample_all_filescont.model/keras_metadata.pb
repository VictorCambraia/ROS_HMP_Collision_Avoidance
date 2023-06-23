
ǡroot"_tf_keras_network*��{"name": "vae", "trainable": true, "expects_training_arg": true, "dtype": "float32", "batch_input_shape": null, "must_restore_from_config": false, "preserve_input_structure_in_config": false, "autocast": false, "class_name": "Functional", "config": {"name": "vae", "trainable": true, "layers": [{"class_name": "InputLayer", "config": {"batch_input_shape": {"class_name": "__tuple__", "items": [null, 1200]}, "dtype": "float32", "sparse": false, "ragged": false, "name": "x_input"}, "name": "x_input", "inbound_nodes": []}, {"class_name": "Flatten", "config": {"name": "flatten", "trainable": true, "dtype": "float32", "data_format": "channels_last"}, "name": "flatten", "inbound_nodes": [[["x_input", 0, 0, {}]]]}, {"class_name": "Dense", "config": {"name": "dense", "trainable": true, "dtype": "float32", "units": 1500, "activation": "relu", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "name": "dense", "inbound_nodes": [[["flatten", 0, 0, {}]]]}, {"class_name": "Dense", "config": {"name": "dense_1", "trainable": true, "dtype": "float32", "units": 500, "activation": "relu", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "name": "dense_1", "inbound_nodes": [[["dense", 0, 0, {}]]]}, {"class_name": "Dense", "config": {"name": "latent_mean_enc", "trainable": true, "dtype": "float32", "units": 50, "activation": "relu", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "name": "latent_mean_enc", "inbound_nodes": [[["dense_1", 0, 0, {}]]]}, {"class_name": "Dense", "config": {"name": "latent_variance_enc", "trainable": true, "dtype": "float32", "units": 50, "activation": "relu", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "name": "latent_variance_enc", "inbound_nodes": [[["dense_1", 0, 0, {}]]]}, {"class_name": "Lambda", "config": {"name": "lambda", "trainable": true, "dtype": "float32", "function": {"class_name": "__tuple__", "items": ["4wEAAAAAAAAAAAAAAAUAAAAFAAAAQwAAAHNiAAAAfABkARkAfQF8AGQCGQB9AnQAagFqAmoDdABq\nAWoCoAR8AaEBZAEZAHQAagFqAqAFfAGhAWQCGQBmAmQDjQF9A3wBdABqAWoCoAZ8AmQEGwChAXwD\nFAAXAH0EfAF9BHwEUwApBU7pAAAAAOkBAAAAKQHaBXNoYXBl6QIAAAApB9oCdGbaBWtlcmFz2gdi\nYWNrZW5k2g1yYW5kb21fbm9ybWFscgMAAADaCWludF9zaGFwZdoDZXhwKQXaBGFyZ3PaBHpfbXXa\nB3pfc2lnbWHaA2Vwc9oBeqkAchAAAAD6HzxpcHl0aG9uLWlucHV0LTEyLWFmYjJiZTA0ZDZhMD7a\nCHNhbXBsZV96BAAAAHMMAAAACAIIATADGgEEAQQC\n", null, null]}, "function_type": "lambda", "module": "__main__", "output_shape": {"class_name": "TensorShape", "items": [null, 50]}, "output_shape_type": "raw", "output_shape_module": null, "arguments": {}}, "name": "lambda", "inbound_nodes": [[["latent_mean_enc", 0, 0, {}], ["latent_variance_enc", 0, 0, {}]]]}, {"class_name": "Dense", "config": {"name": "dense_2", "trainable": true, "dtype": "float32", "units": 100, "activation": "relu", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "name": "dense_2", "inbound_nodes": [[["lambda", 0, 0, {}]]]}, {"class_name": "Dense", "config": {"name": "latent_mean_trans", "trainable": true, "dtype": "float32", "units": 50, "activation": "relu", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "name": "latent_mean_trans", "inbound_nodes": [[["dense_2", 0, 0, {}]]]}, {"class_name": "Dense", "config": {"name": "latent_variance_trans", "trainable": true, "dtype": "float32", "units": 50, "activation": "relu", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "name": "latent_variance_trans", "inbound_nodes": [[["dense_2", 0, 0, {}]]]}, {"class_name": "Lambda", "config": {"name": "lambda_1", "trainable": true, "dtype": "float32", "function": {"class_name": "__tuple__", "items": ["4wEAAAAAAAAAAAAAAAUAAAAFAAAAQwAAAHNiAAAAfABkARkAfQF8AGQCGQB9AnQAagFqAmoDdABq\nAWoCoAR8AaEBZAEZAHQAagFqAqAFfAGhAWQCGQBmAmQDjQF9A3wBdABqAWoCoAZ8AmQEGwChAXwD\nFAAXAH0EfAF9BHwEUwApBU7pAAAAAOkBAAAAKQHaBXNoYXBl6QIAAAApB9oCdGbaBWtlcmFz2gdi\nYWNrZW5k2g1yYW5kb21fbm9ybWFscgMAAADaCWludF9zaGFwZdoDZXhwKQXaBGFyZ3PaBHpfbXXa\nB3pfc2lnbWHaA2Vwc9oBeqkAchAAAAD6HzxpcHl0aG9uLWlucHV0LTEyLWFmYjJiZTA0ZDZhMD7a\nCHNhbXBsZV96BAAAAHMMAAAACAIIATADGgEEAQQC\n", null, null]}, "function_type": "lambda", "module": "__main__", "output_shape": {"class_name": "TensorShape", "items": [null, 50]}, "output_shape_type": "raw", "output_shape_module": null, "arguments": {}}, "name": "lambda_1", "inbound_nodes": [[["latent_mean_trans", 0, 0, {}], ["latent_variance_trans", 0, 0, {}]]]}, {"class_name": "InputLayer", "config": {"batch_input_shape": {"class_name": "__tuple__", "items": [null, 1200]}, "dtype": "float32", "sparse": false, "ragged": false, "name": "y_input"}, "name": "y_input", "inbound_nodes": []}, {"class_name": "Functional", "config": {"name": "decoder", "trainable": true, "layers": [{"class_name": "InputLayer", "config": {"batch_input_shape": {"class_name": "__tuple__", "items": [null, 50]}, "dtype": "float32", "sparse": false, "ragged": false, "name": "input_2"}, "name": "input_2", "inbound_nodes": []}, {"class_name": "Dense", "config": {"name": "dense_3", "trainable": true, "dtype": "float32", "units": 500, "activation": "relu", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "name": "dense_3", "inbound_nodes": [[["input_2", 0, 0, {}]]]}, {"class_name": "Dense", "config": {"name": "dense_4", "trainable": true, "dtype": "float32", "units": 1500, "activation": "relu", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "name": "dense_4", "inbound_nodes": [[["dense_3", 0, 0, {}]]]}, {"class_name": "Dense", "config": {"name": "average_y", "trainable": true, "dtype": "float32", "units": 1200, "activation": "linear", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "name": "average_y", "inbound_nodes": [[["dense_4", 0, 0, {}]]]}, {"class_name": "Dense", "config": {"name": "log_sigma_y", "trainable": true, "dtype": "float32", "units": 1200, "activation": "linear", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "name": "log_sigma_y", "inbound_nodes": [[["dense_4", 0, 0, {}]]]}], "input_layers": [["input_2", 0, 0]], "output_layers": [["average_y", 0, 0], ["log_sigma_y", 0, 0]]}, "name": "decoder", "inbound_nodes": [[["lambda_1", 0, 0, {}]]]}, {"class_name": "CustomLayer", "config": {"name": "custom_layer", "trainable": true, "dtype": "float32"}, "name": "custom_layer", "inbound_nodes": [[["y_input", 0, 0, {}], ["decoder", 1, 0, {}], ["decoder", 1, 1, {}], ["latent_mean_enc", 0, 0, {}], ["latent_variance_enc", 0, 0, {}], ["latent_mean_trans", 0, 0, {}], ["latent_variance_trans", 0, 0, {}]]]}], "input_layers": [["x_input", 0, 0], ["y_input", 0, 0]], "output_layers": {"class_name": "__tuple__", "items": [["custom_layer", 0, 0], ["custom_layer", 0, 1]]}}, "shared_object_id": 41, "input_spec": [{"class_name": "InputSpec", "config": {"dtype": null, "shape": {"class_name": "__tuple__", "items": [null, 1200]}, "ndim": 2, "max_ndim": null, "min_ndim": null, "axes": {}}}, {"class_name": "InputSpec", "config": {"dtype": null, "shape": {"class_name": "__tuple__", "items": [null, 1200]}, "ndim": 2, "max_ndim": null, "min_ndim": null, "axes": {}}}], "build_input_shape": [{"class_name": "TensorShape", "items": [null, 1200]}, {"class_name": "TensorShape", "items": [null, 1200]}], "is_graph_network": true, "full_save_spec": {"class_name": "__tuple__", "items": [[[{"class_name": "TypeSpec", "type_spec": "tf.TensorSpec", "serialized": [{"class_name": "TensorShape", "items": [null, 1200]}, "float32", "x_input"]}, {"class_name": "TypeSpec", "type_spec": "tf.TensorSpec", "serialized": [{"class_name": "TensorShape", "items": [null, 1200]}, "float32", "y_input"]}]], {}]}, "save_spec": [{"class_name": "TypeSpec", "type_spec": "tf.TensorSpec", "serialized": [{"class_name": "TensorShape", "items": [null, 1200]}, "float32", "x_input"]}, {"class_name": "TypeSpec", "type_spec": "tf.TensorSpec", "serialized": [{"class_name": "TensorShape", "items": [null, 1200]}, "float32", "y_input"]}], "keras_version": "2.12.0", "backend": "tensorflow", "model_config": {"class_name": "Functional", "config": {"name": "vae", "trainable": true, "layers": [{"class_name": "InputLayer", "config": {"batch_input_shape": {"class_name": "__tuple__", "items": [null, 1200]}, "dtype": "float32", "sparse": false, "ragged": false, "name": "x_input"}, "name": "x_input", "inbound_nodes": [], "shared_object_id": 0}, {"class_name": "Flatten", "config": {"name": "flatten", "trainable": true, "dtype": "float32", "data_format": "channels_last"}, "name": "flatten", "inbound_nodes": [[["x_input", 0, 0, {}]]], "shared_object_id": 1}, {"class_name": "Dense", "config": {"name": "dense", "trainable": true, "dtype": "float32", "units": 1500, "activation": "relu", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}, "shared_object_id": 2}, "bias_initializer": {"class_name": "Zeros", "config": {}, "shared_object_id": 3}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "name": "dense", "inbound_nodes": [[["flatten", 0, 0, {}]]], "shared_object_id": 4}, {"class_name": "Dense", "config": {"name": "dense_1", "trainable": true, "dtype": "float32", "units": 500, "activation": "relu", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}, "shared_object_id": 5}, "bias_initializer": {"class_name": "Zeros", "config": {}, "shared_object_id": 6}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "name": "dense_1", "inbound_nodes": [[["dense", 0, 0, {}]]], "shared_object_id": 7}, {"class_name": "Dense", "config": {"name": "latent_mean_enc", "trainable": true, "dtype": "float32", "units": 50, "activation": "relu", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}, "shared_object_id": 8}, "bias_initializer": {"class_name": "Zeros", "config": {}, "shared_object_id": 9}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "name": "latent_mean_enc", "inbound_nodes": [[["dense_1", 0, 0, {}]]], "shared_object_id": 10}, {"class_name": "Dense", "config": {"name": "latent_variance_enc", "trainable": true, "dtype": "float32", "units": 50, "activation": "relu", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}, "shared_object_id": 11}, "bias_initializer": {"class_name": "Zeros", "config": {}, "shared_object_id": 12}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "name": "latent_variance_enc", "inbound_nodes": [[["dense_1", 0, 0, {}]]], "shared_object_id": 13}, {"class_name": "Lambda", "config": {"name": "lambda", "trainable": true, "dtype": "float32", "function": {"class_name": "__tuple__", "items": ["4wEAAAAAAAAAAAAAAAUAAAAFAAAAQwAAAHNiAAAAfABkARkAfQF8AGQCGQB9AnQAagFqAmoDdABq\nAWoCoAR8AaEBZAEZAHQAagFqAqAFfAGhAWQCGQBmAmQDjQF9A3wBdABqAWoCoAZ8AmQEGwChAXwD\nFAAXAH0EfAF9BHwEUwApBU7pAAAAAOkBAAAAKQHaBXNoYXBl6QIAAAApB9oCdGbaBWtlcmFz2gdi\nYWNrZW5k2g1yYW5kb21fbm9ybWFscgMAAADaCWludF9zaGFwZdoDZXhwKQXaBGFyZ3PaBHpfbXXa\nB3pfc2lnbWHaA2Vwc9oBeqkAchAAAAD6HzxpcHl0aG9uLWlucHV0LTEyLWFmYjJiZTA0ZDZhMD7a\nCHNhbXBsZV96BAAAAHMMAAAACAIIATADGgEEAQQC\n", null, null]}, "function_type": "lambda", "module": "__main__", "output_shape": {"class_name": "TensorShape", "items": [null, 50]}, "output_shape_type": "raw", "output_shape_module": null, "arguments": {}}, "name": "lambda", "inbound_nodes": [[["latent_mean_enc", 0, 0, {}], ["latent_variance_enc", 0, 0, {}]]], "shared_object_id": 14}, {"class_name": "Dense", "config": {"name": "dense_2", "trainable": true, "dtype": "float32", "units": 100, "activation": "relu", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}, "shared_object_id": 15}, "bias_initializer": {"class_name": "Zeros", "config": {}, "shared_object_id": 16}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "name": "dense_2", "inbound_nodes": [[["lambda", 0, 0, {}]]], "shared_object_id": 17}, {"class_name": "Dense", "config": {"name": "latent_mean_trans", "trainable": true, "dtype": "float32", "units": 50, "activation": "relu", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}, "shared_object_id": 18}, "bias_initializer": {"class_name": "Zeros", "config": {}, "shared_object_id": 19}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "name": "latent_mean_trans", "inbound_nodes": [[["dense_2", 0, 0, {}]]], "shared_object_id": 20}, {"class_name": "Dense", "config": {"name": "latent_variance_trans", "trainable": true, "dtype": "float32", "units": 50, "activation": "relu", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}, "shared_object_id": 21}, "bias_initializer": {"class_name": "Zeros", "config": {}, "shared_object_id": 22}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "name": "latent_variance_trans", "inbound_nodes": [[["dense_2", 0, 0, {}]]], "shared_object_id": 23}, {"class_name": "Lambda", "config": {"name": "lambda_1", "trainable": true, "dtype": "float32", "function": {"class_name": "__tuple__", "items": ["4wEAAAAAAAAAAAAAAAUAAAAFAAAAQwAAAHNiAAAAfABkARkAfQF8AGQCGQB9AnQAagFqAmoDdABq\nAWoCoAR8AaEBZAEZAHQAagFqAqAFfAGhAWQCGQBmAmQDjQF9A3wBdABqAWoCoAZ8AmQEGwChAXwD\nFAAXAH0EfAF9BHwEUwApBU7pAAAAAOkBAAAAKQHaBXNoYXBl6QIAAAApB9oCdGbaBWtlcmFz2gdi\nYWNrZW5k2g1yYW5kb21fbm9ybWFscgMAAADaCWludF9zaGFwZdoDZXhwKQXaBGFyZ3PaBHpfbXXa\nB3pfc2lnbWHaA2Vwc9oBeqkAchAAAAD6HzxpcHl0aG9uLWlucHV0LTEyLWFmYjJiZTA0ZDZhMD7a\nCHNhbXBsZV96BAAAAHMMAAAACAIIATADGgEEAQQC\n", null, null]}, "function_type": "lambda", "module": "__main__", "output_shape": {"class_name": "TensorShape", "items": [null, 50]}, "output_shape_type": "raw", "output_shape_module": null, "arguments": {}}, "name": "lambda_1", "inbound_nodes": [[["latent_mean_trans", 0, 0, {}], ["latent_variance_trans", 0, 0, {}]]], "shared_object_id": 24}, {"class_name": "InputLayer", "config": {"batch_input_shape": {"class_name": "__tuple__", "items": [null, 1200]}, "dtype": "float32", "sparse": false, "ragged": false, "name": "y_input"}, "name": "y_input", "inbound_nodes": [], "shared_object_id": 25}, {"class_name": "Functional", "config": {"name": "decoder", "trainable": true, "layers": [{"class_name": "InputLayer", "config": {"batch_input_shape": {"class_name": "__tuple__", "items": [null, 50]}, "dtype": "float32", "sparse": false, "ragged": false, "name": "input_2"}, "name": "input_2", "inbound_nodes": []}, {"class_name": "Dense", "config": {"name": "dense_3", "trainable": true, "dtype": "float32", "units": 500, "activation": "relu", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "name": "dense_3", "inbound_nodes": [[["input_2", 0, 0, {}]]]}, {"class_name": "Dense", "config": {"name": "dense_4", "trainable": true, "dtype": "float32", "units": 1500, "activation": "relu", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "name": "dense_4", "inbound_nodes": [[["dense_3", 0, 0, {}]]]}, {"class_name": "Dense", "config": {"name": "average_y", "trainable": true, "dtype": "float32", "units": 1200, "activation": "linear", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "name": "average_y", "inbound_nodes": [[["dense_4", 0, 0, {}]]]}, {"class_name": "Dense", "config": {"name": "log_sigma_y", "trainable": true, "dtype": "float32", "units": 1200, "activation": "linear", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "name": "log_sigma_y", "inbound_nodes": [[["dense_4", 0, 0, {}]]]}], "input_layers": [["input_2", 0, 0]], "output_layers": [["average_y", 0, 0], ["log_sigma_y", 0, 0]]}, "name": "decoder", "inbound_nodes": [[["lambda_1", 0, 0, {}]]], "shared_object_id": 39}, {"class_name": "CustomLayer", "config": {"name": "custom_layer", "trainable": true, "dtype": "float32"}, "name": "custom_layer", "inbound_nodes": [[["y_input", 0, 0, {}], ["decoder", 1, 0, {}], ["decoder", 1, 1, {}], ["latent_mean_enc", 0, 0, {}], ["latent_variance_enc", 0, 0, {}], ["latent_mean_trans", 0, 0, {}], ["latent_variance_trans", 0, 0, {}]]], "shared_object_id": 40}], "input_layers": [["x_input", 0, 0], ["y_input", 0, 0]], "output_layers": {"class_name": "__tuple__", "items": [["custom_layer", 0, 0], ["custom_layer", 0, 1]]}}}, "training_config": {"loss": null, "metrics": null, "weighted_metrics": null, "loss_weights": null, "optimizer_config": {"class_name": "Custom>Adam", "config": {"name": "Adam", "weight_decay": null, "clipnorm": null, "global_clipnorm": null, "clipvalue": null, "use_ema": false, "ema_momentum": 0.99, "ema_overwrite_frequency": null, "jit_compile": true, "is_legacy_optimizer": false, "learning_rate": 0.0010000000474974513, "beta_1": 0.95, "beta_2": 0.999, "epsilon": 1e-08, "amsgrad": false}}}}2
�root.layer-0"_tf_keras_input_layer*�{"class_name": "InputLayer", "name": "x_input", "dtype": "float32", "sparse": false, "ragged": false, "batch_input_shape": {"class_name": "__tuple__", "items": [null, 1200]}, "config": {"batch_input_shape": {"class_name": "__tuple__", "items": [null, 1200]}, "dtype": "float32", "sparse": false, "ragged": false, "name": "x_input"}}2
�root.layer-1"_tf_keras_layer*�{"name": "flatten", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "preserve_input_structure_in_config": false, "autocast": true, "class_name": "Flatten", "config": {"name": "flatten", "trainable": true, "dtype": "float32", "data_format": "channels_last"}, "inbound_nodes": [[["x_input", 0, 0, {}]]], "shared_object_id": 1, "input_spec": {"class_name": "InputSpec", "config": {"dtype": null, "shape": null, "ndim": null, "max_ndim": null, "min_ndim": 1, "axes": {}}, "shared_object_id": 44}, "build_input_shape": {"class_name": "TensorShape", "items": [null, 1200]}}2
�root.layer_with_weights-0"_tf_keras_layer*�{"name": "dense", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "preserve_input_structure_in_config": false, "autocast": true, "class_name": "Dense", "config": {"name": "dense", "trainable": true, "dtype": "float32", "units": 1500, "activation": "relu", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}, "shared_object_id": 2}, "bias_initializer": {"class_name": "Zeros", "config": {}, "shared_object_id": 3}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "inbound_nodes": [[["flatten", 0, 0, {}]]], "shared_object_id": 4, "input_spec": {"class_name": "InputSpec", "config": {"dtype": null, "shape": null, "ndim": null, "max_ndim": null, "min_ndim": 2, "axes": {"-1": 1200}}, "shared_object_id": 45}, "build_input_shape": {"class_name": "TensorShape", "items": [null, 1200]}}2
�root.layer_with_weights-1"_tf_keras_layer*�{"name": "dense_1", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "preserve_input_structure_in_config": false, "autocast": true, "class_name": "Dense", "config": {"name": "dense_1", "trainable": true, "dtype": "float32", "units": 500, "activation": "relu", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}, "shared_object_id": 5}, "bias_initializer": {"class_name": "Zeros", "config": {}, "shared_object_id": 6}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "inbound_nodes": [[["dense", 0, 0, {}]]], "shared_object_id": 7, "input_spec": {"class_name": "InputSpec", "config": {"dtype": null, "shape": null, "ndim": null, "max_ndim": null, "min_ndim": 2, "axes": {"-1": 1500}}, "shared_object_id": 46}, "build_input_shape": {"class_name": "TensorShape", "items": [null, 1500]}}2
�root.layer_with_weights-2"_tf_keras_layer*�{"name": "latent_mean_enc", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "preserve_input_structure_in_config": false, "autocast": true, "class_name": "Dense", "config": {"name": "latent_mean_enc", "trainable": true, "dtype": "float32", "units": 50, "activation": "relu", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}, "shared_object_id": 8}, "bias_initializer": {"class_name": "Zeros", "config": {}, "shared_object_id": 9}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "inbound_nodes": [[["dense_1", 0, 0, {}]]], "shared_object_id": 10, "input_spec": {"class_name": "InputSpec", "config": {"dtype": null, "shape": null, "ndim": null, "max_ndim": null, "min_ndim": 2, "axes": {"-1": 500}}, "shared_object_id": 47}, "build_input_shape": {"class_name": "TensorShape", "items": [null, 500]}}2
�root.layer_with_weights-3"_tf_keras_layer*�{"name": "latent_variance_enc", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "preserve_input_structure_in_config": false, "autocast": true, "class_name": "Dense", "config": {"name": "latent_variance_enc", "trainable": true, "dtype": "float32", "units": 50, "activation": "relu", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}, "shared_object_id": 11}, "bias_initializer": {"class_name": "Zeros", "config": {}, "shared_object_id": 12}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "inbound_nodes": [[["dense_1", 0, 0, {}]]], "shared_object_id": 13, "input_spec": {"class_name": "InputSpec", "config": {"dtype": null, "shape": null, "ndim": null, "max_ndim": null, "min_ndim": 2, "axes": {"-1": 500}}, "shared_object_id": 48}, "build_input_shape": {"class_name": "TensorShape", "items": [null, 500]}}2
�
root.layer-6"_tf_keras_layer*�	{"name": "lambda", "trainable": true, "expects_training_arg": true, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "preserve_input_structure_in_config": false, "autocast": true, "class_name": "Lambda", "config": {"name": "lambda", "trainable": true, "dtype": "float32", "function": {"class_name": "__tuple__", "items": ["4wEAAAAAAAAAAAAAAAUAAAAFAAAAQwAAAHNiAAAAfABkARkAfQF8AGQCGQB9AnQAagFqAmoDdABq\nAWoCoAR8AaEBZAEZAHQAagFqAqAFfAGhAWQCGQBmAmQDjQF9A3wBdABqAWoCoAZ8AmQEGwChAXwD\nFAAXAH0EfAF9BHwEUwApBU7pAAAAAOkBAAAAKQHaBXNoYXBl6QIAAAApB9oCdGbaBWtlcmFz2gdi\nYWNrZW5k2g1yYW5kb21fbm9ybWFscgMAAADaCWludF9zaGFwZdoDZXhwKQXaBGFyZ3PaBHpfbXXa\nB3pfc2lnbWHaA2Vwc9oBeqkAchAAAAD6HzxpcHl0aG9uLWlucHV0LTEyLWFmYjJiZTA0ZDZhMD7a\nCHNhbXBsZV96BAAAAHMMAAAACAIIATADGgEEAQQC\n", null, null]}, "function_type": "lambda", "module": "__main__", "output_shape": {"class_name": "TensorShape", "items": [null, 50]}, "output_shape_type": "raw", "output_shape_module": null, "arguments": {}}, "inbound_nodes": [[["latent_mean_enc", 0, 0, {}], ["latent_variance_enc", 0, 0, {}]]], "shared_object_id": 14, "build_input_shape": [{"class_name": "TensorShape", "items": [null, 50]}, {"class_name": "TensorShape", "items": [null, 50]}]}2
�root.layer_with_weights-4"_tf_keras_layer*�{"name": "dense_2", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "preserve_input_structure_in_config": false, "autocast": true, "class_name": "Dense", "config": {"name": "dense_2", "trainable": true, "dtype": "float32", "units": 100, "activation": "relu", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}, "shared_object_id": 15}, "bias_initializer": {"class_name": "Zeros", "config": {}, "shared_object_id": 16}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "inbound_nodes": [[["lambda", 0, 0, {}]]], "shared_object_id": 17, "input_spec": {"class_name": "InputSpec", "config": {"dtype": null, "shape": null, "ndim": null, "max_ndim": null, "min_ndim": 2, "axes": {"-1": 50}}, "shared_object_id": 49}, "build_input_shape": {"class_name": "TensorShape", "items": [null, 50]}}2
�	root.layer_with_weights-5"_tf_keras_layer*�{"name": "latent_mean_trans", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "preserve_input_structure_in_config": false, "autocast": true, "class_name": "Dense", "config": {"name": "latent_mean_trans", "trainable": true, "dtype": "float32", "units": 50, "activation": "relu", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}, "shared_object_id": 18}, "bias_initializer": {"class_name": "Zeros", "config": {}, "shared_object_id": 19}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "inbound_nodes": [[["dense_2", 0, 0, {}]]], "shared_object_id": 20, "input_spec": {"class_name": "InputSpec", "config": {"dtype": null, "shape": null, "ndim": null, "max_ndim": null, "min_ndim": 2, "axes": {"-1": 100}}, "shared_object_id": 50}, "build_input_shape": {"class_name": "TensorShape", "items": [null, 100]}}2
�
root.layer_with_weights-6"_tf_keras_layer*�{"name": "latent_variance_trans", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "preserve_input_structure_in_config": false, "autocast": true, "class_name": "Dense", "config": {"name": "latent_variance_trans", "trainable": true, "dtype": "float32", "units": 50, "activation": "relu", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}, "shared_object_id": 21}, "bias_initializer": {"class_name": "Zeros", "config": {}, "shared_object_id": 22}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "inbound_nodes": [[["dense_2", 0, 0, {}]]], "shared_object_id": 23, "input_spec": {"class_name": "InputSpec", "config": {"dtype": null, "shape": null, "ndim": null, "max_ndim": null, "min_ndim": 2, "axes": {"-1": 100}}, "shared_object_id": 51}, "build_input_shape": {"class_name": "TensorShape", "items": [null, 100]}}2
�

�
�0
�
�c!root.layer_with_weights-7.layer-0"_tf_keras_input_layer*�{"class_name": "InputLayer", "name": "input_2", "dtype": "float32", "sparse": false, "ragged": false, "batch_input_shape": {"class_name": "__tuple__", "items": [null, 50]}, "config": {"batch_input_shape": {"class_name": "__tuple__", "items": [null, 50]}, "dtype": "float32", "sparse": false, "ragged": false, "name": "input_2"}}2
�d.root.layer_with_weights-7.layer_with_weights-0"_tf_keras_layer*�{"name": "dense_3", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "preserve_input_structure_in_config": false, "autocast": true, "class_name": "Dense", "config": {"name": "dense_3", "trainable": true, "dtype": "float32", "units": 500, "activation": "relu", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}, "shared_object_id": 27}, "bias_initializer": {"class_name": "Zeros", "config": {}, "shared_object_id": 28}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "inbound_nodes": [[["input_2", 0, 0, {}]]], "shared_object_id": 29, "input_spec": {"class_name": "InputSpec", "config": {"dtype": null, "shape": null, "ndim": null, "max_ndim": null, "min_ndim": 2, "axes": {"-1": 50}}, "shared_object_id": 53}, "build_input_shape": {"class_name": "TensorShape", "items": [null, 50]}}2
�e.root.layer_with_weights-7.layer_with_weights-1"_tf_keras_layer*�{"name": "dense_4", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "preserve_input_structure_in_config": false, "autocast": true, "class_name": "Dense", "config": {"name": "dense_4", "trainable": true, "dtype": "float32", "units": 1500, "activation": "relu", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}, "shared_object_id": 30}, "bias_initializer": {"class_name": "Zeros", "config": {}, "shared_object_id": 31}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "inbound_nodes": [[["dense_3", 0, 0, {}]]], "shared_object_id": 32, "input_spec": {"class_name": "InputSpec", "config": {"dtype": null, "shape": null, "ndim": null, "max_ndim": null, "min_ndim": 2, "axes": {"-1": 500}}, "shared_object_id": 54}, "build_input_shape": {"class_name": "TensorShape", "items": [null, 500]}}2
�f.root.layer_with_weights-7.layer_with_weights-2"_tf_keras_layer*�{"name": "average_y", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "preserve_input_structure_in_config": false, "autocast": true, "class_name": "Dense", "config": {"name": "average_y", "trainable": true, "dtype": "float32", "units": 1200, "activation": "linear", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}, "shared_object_id": 33}, "bias_initializer": {"class_name": "Zeros", "config": {}, "shared_object_id": 34}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "inbound_nodes": [[["dense_4", 0, 0, {}]]], "shared_object_id": 35, "input_spec": {"class_name": "InputSpec", "config": {"dtype": null, "shape": null, "ndim": null, "max_ndim": null, "min_ndim": 2, "axes": {"-1": 1500}}, "shared_object_id": 55}, "build_input_shape": {"class_name": "TensorShape", "items": [null, 1500]}}2
�g.root.layer_with_weights-7.layer_with_weights-3"_tf_keras_layer*�{"name": "log_sigma_y", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "preserve_input_structure_in_config": false, "autocast": true, "class_name": "Dense", "config": {"name": "log_sigma_y", "trainable": true, "dtype": "float32", "units": 1200, "activation": "linear", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}, "shared_object_id": 36}, "bias_initializer": {"class_name": "Zeros", "config": {}, "shared_object_id": 37}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "inbound_nodes": [[["dense_4", 0, 0, {}]]], "shared_object_id": 38, "input_spec": {"class_name": "InputSpec", "config": {"dtype": null, "shape": null, "ndim": null, "max_ndim": null, "min_ndim": 2, "axes": {"-1": 1500}}, "shared_object_id": 56}, "build_input_shape": {"class_name": "TensorShape", "items": [null, 1500]}}2
��root.keras_api.metrics.0"_tf_keras_metric*�{"class_name": "Mean", "name": "loss", "dtype": "float32", "config": {"name": "loss", "dtype": "float32"}, "shared_object_id": 57}2