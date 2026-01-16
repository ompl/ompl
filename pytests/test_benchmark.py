
import ompl.tools as tools

def test_run_properties():
    print("Testing RunProperties...")
    try:
        props = tools.RunProperties()
        
        # Test setting items
        props["test_key"] = "test_value"
        print("Set item successful")
        
        # Test getting items
        val = props["test_key"]
        print(f"Get item successful: {val}")
        assert val == "test_value"
        
        # Test length
        l = len(props)
        print(f"Length: {l}")
        assert l == 1
        
        # Test contains
        exists = "test_key" in props
        print(f"Contains 'test_key': {exists}")
        assert exists
        
        not_exists = "foo" in props
        print(f"Contains 'foo': {not_exists}")
        assert not not_exists
        
        # Test to_dict
        d = props.to_dict()
        print(f"to_dict: {d}")
        assert isinstance(d, dict)
        assert d["test_key"] == "test_value"
        
        print("RunProperties verification passed!")
        
    except Exception as e:
        print(f"RunProperties verification FAILED: {e}")
        raise

if __name__ == "__main__":
    test_run_properties()
