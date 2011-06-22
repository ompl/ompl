<?php 
$your_email ='ompl-devel@lists.sourceforge.net';// <<=== update to your email address

session_start();
$errors = '';
$name = '';
$visitor_email = '';
$user_message = '';

if(isset($_POST['submit']))
{
	
	$name = $_POST['name'];
	$visitor_email = $_POST['email'];
	$user_message = $_POST['message'];
	///------------Do Validations-------------
	if(empty($name)||empty($visitor_email))
	{
		$errors .= "\n Name and Email are required fields. ";	
	}
	if(IsInjected($visitor_email))
	{
		$errors .= "\n Bad email value!";
	}
	if(empty($_SESSION['6_letters_code'] ) ||
	  strcasecmp($_SESSION['6_letters_code'], $_POST['6_letters_code']) != 0)
	{
	//Note: the captcha code is compared case insensitively.
	//if you want case sensitive match, update the check above to
	// strcmp()
		$errors .= "\n The captcha code does not match!";
	}
	
	if(empty($errors))
	{
		//send the email
		$to = $your_email;
		$subject="New form submission";
		$from = $your_email;
		$ip = isset($_SERVER['REMOTE_ADDR']) ? $_SERVER['REMOTE_ADDR'] : '';
		
		$body = "A user  $name submitted the contact form:\n".
		"Name: $name\n".
		"Email: $visitor_email \n".
		"Message: \n ".
		"$user_message\n".
		"IP: $ip\n";	
		
		$headers = "From: $from \r\n";
		$headers .= "Reply-To: $visitor_email \r\n";
		
		mail($to, $subject, $body,$headers);
		
		header('Location: thank-you.html');
	}
}

// Function to validate against any email injection attempts
function IsInjected($str)
{
  $injections = array('(\n+)',
              '(\r+)',
              '(\t+)',
              '(%0A+)',
              '(%0D+)',
              '(%08+)',
              '(%09+)'
              );
  $inject = join('|', $injections);
  $inject = "/$inject/i";
  if(preg_match($inject,$str))
    {
    return true;
  }
  else
    {
    return false;
  }
}
?>
<!DOCTYPE HTML PUBLIC "-//W3C//DTD HTML 4.01 Transitional//EN" "http://www.w3.org/TR/html4/loose.dtd"> 
<html>
<head>
	<title>Contact Us</title>
  <link rel="stylesheet" href="../css/screen.css" type="text/css" media="screen, projection">
  <link rel="stylesheet" href="../css/print.css" type="text/css" media="print">
<!-- define some style elements-->
<style>
.err { color: #f00; }
textarea { resize: none; }
</style>	
<!-- a helper script for vaidating the form-->
<script language="JavaScript" src="../js/gen_validatorv31.js" type="text/javascript"></script>	
</head>

<body>
<?php
if(!empty($errors)){
echo "<p class='err'>".nl2br($errors)."</p>";
}
?>
<div id='contact_form_errorloc' class="err span-11 last"></div>
<form method="POST" name="contact_form"
action="<?php echo htmlentities($_SERVER['PHP_SELF']); ?>" class="span-11 last">
<div class="span-6"><label for='name'>Name: </label></div>
<div class="span-5 last"><label for='email'>Email: </label></div>
<div class="span-5"><input type="text" name="name" value='<?php echo htmlentities($name) ?>' class="span-5 last"></div>
<div class="push-1 span-5 last"><input type="text" name="email" value='<?php echo htmlentities($visitor_email) ?>' class="span-5 last"></div>
<label for='message' class="span-11 last">Message:</label>
<textarea name="message" rows=18 cols=20 class="span-11 last"><?php echo htmlentities($user_message) ?></textarea>
<img src="captcha_code_file.php?rand=<?php echo rand(); ?>" id='captchaimg' ><br>
<div class="span-11 last"><label for='message'>Enter the code above here :</label></div>
<div class="span-11 last"><input id="6_letters_code" name="6_letters_code" type="text" class="span-5"></div>
<small>Can't read the image? click <a href='javascript: refreshCaptcha();'>here</a> to refresh</small>
</p>
<input type="submit" value="Submit" name='submit'>
</form>
</div>
<script language="JavaScript">
// Code for validating the form
// Visit http://www.javascript-coder.com/html-form/javascript-form-validation.phtml
// for details
var frmvalidator  = new Validator("contact_form");
//remove the following two lines if you like error message box popups
frmvalidator.EnableOnPageErrorDisplaySingleBox();
frmvalidator.EnableMsgsTogether();

frmvalidator.addValidation("name","req","Please provide your name"); 
frmvalidator.addValidation("email","req","Please provide your email"); 
frmvalidator.addValidation("email","email","Please enter a valid email address"); 
</script>
<script language='JavaScript' type='text/javascript'>
function refreshCaptcha()
{
	var img = document.images['captchaimg'];
	img.src = img.src.substring(0,img.src.lastIndexOf("?"))+"?rand="+Math.random()*1000;
}
</script>
</body>
</html>