from tensorflow.python.compiler.tensorrt import trt_convert as trt

input_saved_model_dir = "signal_5_ligero"
converter = trt.TrtGraphConverterV2(input_saved_model_dir=input_saved_model_dir)
converter.convert()
converter.save("signal_5_ligero_RT")
